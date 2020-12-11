from cubic_spline_planner import *
from casadi import *
from casadi.tools import *
from pylab import *
import numpy as np
from ros_mpc import *
from colorline import *
from scipy import signal

# track best path optimization 

def opt_track(xs, ys, sins, coss, widths_min, widths_max):

    #MAX_A  = 0.3  # max throttle acc
    #MAX_B  = 0.05  # max brake acc
    #MAX_AL = 0.09
    #MAX_SPEED = 3

    MAX_A  = 0.5  # max throttle acc
    MAX_B  = 0.05  # max brake acc
    MAX_AL = 0.07
    MAX_SPEED = 5

    #MAX_A  = 0.5  # max throttle acc
    #MAX_B  = 0.05  # max brake acc
    #MAX_AL = 0.15
    #MAX_SPEED = 5

    N = len(xs)

    U  = MX.sym("x", N)
    Us = MX.sym("speeds", N) 
    J = 0

    ks = []
    ds = []
    dt = []
    for i in range(N,N*2):
        i1 = (i-1) % N
        i2 = (i) % N
        i3 = (i+1) % N

        #  (i1)
        #  A ----- B (i2)
        #           \
        #            C (i3)
        #
        x1 = xs[i1] + U[i1]*coss[i1]
        y1 = ys[i1] + U[i1]*sins[i1]  
        x2 = xs[i2] + U[i2]*coss[i2]
        y2 = ys[i2] + U[i2]*sins[i2]  
        x3 = xs[i3] + U[i3]*coss[i3]
        y3 = ys[i3] + U[i3]*sins[i3]  
       
        # curvature
        # https://gis.stackexchange.com/questions/195370/determining-curvature-of-polylines
        k = 2*((x2-x1)*(y3-y2)-(y2-y1)*(x3-x2)) / sqrt( ((x2-x1)**2+(y2-y1)**2) * ((x3-x2)**2+(y3-y2)**2) * ((x1-x3)**2+(y1-y3)**2) )

        ks.append(k)
        ds.append( sqrt( (x2 - x3)**2 + (y2 -y3)**2 ) )

        # min curv
        J += 10000*k**2
        # max speed
        J += (MAX_SPEED - Us[i2])/MAX_SPEED
        # min time
        dt.append(ds[i2] / Us[i2])
        J += dt[i2]
        
    #G = ks[0] - ks[-1]
    G = 0   # k vincolo
    Gal = 0  # speed vincolo 
    Ga = 0  # acc vincolo
    for i in range(N):
        i  = i
        i2 = (i+1) % N

        #dk = (ks[i] - ks[iB])**2
        #J += dk

        k = ks[i]

        al = k * (Us[i]**2)
        a  = (Us[i2]**2 - Us[i]**2) / (2*ds[i])

        J += a*a
        # min lat acc derivate 
        #J += ( k*( (Us[i]**2) - (Us[i2]**2) ) )**2

        #J += 0.1*((a**2)*(al**2))

        if(i == 0):
            G = k
            Gal = al
            Ga = a
        else:
            G = vertcat( G, k )
            Gal = vertcat( Gal, al )
            Ga = vertcat( Ga, a )
    
    # NLP
    nlp = {'x':vertcat(U,Us), 'f':J, 'g':vertcat(G, Gal, Ga)}
    
    # Allocate an NLP solver
    opts = {"ipopt.tol":1e-10, "expand":True, "ipopt.max_iter": 1000}
    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(widths_min, 0*np.ones(N))
    arg["ubx"] =  vertcat(widths_max, MAX_SPEED*np.ones(N))
    arg["x0"] =   vertcat(np.ones(N)*0, np.ones(N)*MAX_SPEED)

    # Bounds on g              max curv      max acc l          max acc
    arg["lbg"] =  vertcat( -0.1*np.ones(N), -MAX_AL*np.ones(N), -MAX_B*np.ones(N) )
    arg["ubg"] =  vertcat(  0.1*np.ones(N), +MAX_AL*np.ones(N), +MAX_A*np.ones(N) )

    # Solve the problem
    res = solver(**arg)

    print res
    print "TIME: ", res["f"]
    return res


if __name__ == "__main__":
    # spline to follow
    xs, ys, zs = load_flag_path("tracks/marzaglia.txt")
    xs.append(xs[0])
    ys.append(ys[0])
    zs.append(zs[0])

    print xs
    print ys
    path = Spline2D(xs, ys)

    # path point to opt
    N = 600
    TRACK_W = 6

    s = np.linspace(0, path.s[-1], N+1)[:-1]
    # center points
    xs = [ path.calc_position(i)[0] for i in s ]
    ys = [ path.calc_position(i)[1] for i in s ]

    yaws = [ path.calc_yaw(i) + np.pi/2 for i in s ]
    sins = [ sin(i) for i in yaws]
    coss = [ cos(i) for i in yaws]

    # inside points
    xs_s = [ xs[i] + coss[i]*(-TRACK_W) for i in range(len(yaws)) ]
    ys_s = [ ys[i] + sins[i]*(-TRACK_W) for i in range(len(yaws)) ]
    xs_s.append(xs_s[0])
    ys_s.append(ys_s[0])

    # outside points
    xs_f = [ xs[i] + coss[i]*TRACK_W for i in range(len(yaws)) ]
    ys_f = [ ys[i] + sins[i]*TRACK_W for i in range(len(yaws)) ]
    xs_f.append(xs_f[0])
    ys_f.append(ys_f[0])

    
    res = opt_track(xs, ys, sins, coss, -np.ones(N)*(TRACK_W-1.5), np.ones(N)*(TRACK_W-1.5))
    sol = res["x"]
    time = res["f"]
    g = res["g"]

    # plot solution

    # append for close path
    xs.append(xs[0])
    ys.append(ys[0])

    xs_sol = [ xs[i] + coss[i]*sol[i] for i in range(len(yaws)) ]
    ys_sol = [ ys[i] + sins[i]*sol[i] for i in range(len(yaws)) ]
    xs_sol.append(xs_sol[0])
    ys_sol.append(ys_sol[0])

    # splinize solution and calc curvature
    path_sol = Spline2D(xs_sol, ys_sol)
    s = np.concatenate([ np.linspace(0, path_sol.s[-1], N)[:-1], [0] ])
    xs_sol = [ path_sol.calc_position(i)[0] for i in s ]
    ys_sol = [ path_sol.calc_position(i)[1] for i in s ]
    
    speeds = [ sol[N+i]*3.6 for i in xrange(N)]
    max_speed = max(speeds)
    print "min/max speeds: ", min(speeds), max(speeds)     

    peakind_max = signal.find_peaks_cwt(speeds, np.arange(1,20))
    peakind_min = signal.find_peaks_cwt([-_s for _s in speeds], np.arange(1,20))
    peakind = np.concatenate([ peakind_min, peakind_max ])
    #print peakind
    times = linspace(0, time, N)
    
    subplot(222)
    plot(times, speeds)
    plot([times[i] for i in peakind_min], [speeds[i] for i in peakind_min], 'o')
    plot([times[i] for i in peakind_max], [speeds[i] for i in peakind_max], 'x')
    title( ("speed") )

    subplot(224)
    plot(times, [ float(g[i]) for i in range(N, N*2)])
    title( ("lateral acceleration") )

    subplot(121)
    for i in xrange(N):
        plot([xs_s[i], xs_f[i]], [ys_s[i], ys_f[i]], '0.75')
    plot(xs_s, ys_s, '0.5')
    plot(xs_f, ys_f, '0.5')

    save_flag_path("tracks/marzaglia_opt.txt", xs_sol, ys_sol, [sol[N+i] for i in range(N)])

    speeds_norm = [ s/max_speed for s in speeds]
    xs_sol.append(xs_sol[0])
    ys_sol.append(ys_sol[0])
    speeds.append(speeds[0])
    colorline(xs_sol, ys_sol, z=speeds_norm)
    axis('equal')
    title( ("track") )

    # show speed labels
    for i in peakind:
        text(xs[i] + 10, ys[i], str(int(speeds[i])) + " kmh", fontsize=10, zorder=20)

    show()
