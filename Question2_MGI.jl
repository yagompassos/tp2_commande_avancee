using Optim
using LinearAlgebra
using Printf

# connexion avec VREP
const current_dir = @__DIR__
PATHCSIM = joinpath(current_dir, "lib")
include(joinpath(PATHCSIM, "lib-robotique.jl"))
include(joinpath(PATHCSIM, "Lib-CSim.jl"))
global clientID=startsimulation(simx_opmode_oneshot)
if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

global rob=CreateRobotKukaLwr();
θmin =[-1.2,-1.2,-1.2,-1.2,-1.2,-1.2,-1.2]
θmax = [1.0,1.0,1.0,1.0,1.0,1.0,1.0]
Pos_init = [-0.3668, -0.0379, 0.8634]
Pos_ref =  [0.0, -0.0379, 0.5]
Pos_ref_innateignable = [5.0, 20.0, 0.5]

println("Mouvement pos initiale...")
θ_init=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868]
init_pos()

T = MGD(θ_init, rob)
print("Position initiale = ", round.(T[1:3, 4],digits=4)) # Verifié comme etant la bonne Pos_init.

# fonction pour chercher la position articulaire de reference
println("\n1. Optimisation pour la pos ref...")
function f(θ)
    T = MGD(θ, rob)
    Pos_actuel = T[1:3, 4] # 1x3 (vecteur position qui vient de la matr de Transform T)
    return norm(Pos_ref - Pos_actuel)^2
end

# @time results=optimize(f, θ_init, LBFGS()) # <---------------- SANS LIMITES
@time results=optimize(f, θmin, θmax, θ_init, IPNewton() )
theta=Optim.minimizer(results)

println("2. Valeur trouvé!\nMouvement vers le Pos_ref...")
@time controlrobot(theta, clientID, simx_opmode_oneshot, simx_opmode_oneshot, 7, objectname_kuka)
sleep(2)


T = MGD(theta, rob)
ecart = round.(Pos_ref - T[1:3,4], digits=4)
println("3. Validation!\npos ref théorique = ", round.(T[1:3,4], digits=4))
@printf "ecart theorique (Pos_ref - Pos_optm) = [%.4f, %.4f, %.4f]\n" ecart[1] ecart[2] ecart[3]

stopsimulation(clientID, simx_opmode_oneshot)