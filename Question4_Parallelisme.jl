using Optim
using LinearAlgebra

# Connection CoppeliaSim
const current_dir = @__DIR__
PATHCSIM = joinpath(current_dir, "lib")
include(joinpath(PATHCSIM, "lib-robotique.jl"))
include(joinpath(PATHCSIM, "Lib-CSim.jl"))

global clientID = startsimulation(simx_opmode_oneshot)

# init robot
global rob = CreateRobotKukaLwr()
θ_init = [0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868]
init_pos()

global Pos_ref = [0.0, -0.0379, 0.5]
global θ_cmd = copy(θ_init)

# Temps mesurées en 4.a)
global dt_chemin = 0.2 # 200ms 
global dt_mgi = 0.02  # 20ms
global dt_comms = 0.08 # 80ms

# tache 1 : MGI
function task_mgi(::Timer)
    f(q) = norm(Pos_ref - MGD(q, rob)[1:3, 4])^2
    res = optimize(f, θ_cmd, LBFGS())
    global θ_cmd = Optim.minimizer(res)
end

# Tache 2 : envoi vers coppelia
function task_coppelia(::Timer)
    controlrobot(θ_cmd, clientID, simx_opmode_oneshot, simx_opmode_oneshot, 7, objectname_kuka)
end

global timer_mgi = Timer(task_mgi, 0.0, interval = dt_mgi)
global timer_comms = Timer(task_coppelia, 0.0, interval = dt_comms)

sleep(30.0)
close(timer_mgi)
close(timer_comms)
stopsimulation(clientID, simx_opmode_oneshot)