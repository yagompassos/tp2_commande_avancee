using Optim
using LinearAlgebra

# connexion avec VREP
const current_dir = @__DIR__
PATHCSIM = joinpath(current_dir, "lib")
include(joinpath(PATHCSIM, "lib-robotique.jl"))
include(joinpath(PATHCSIM, "Lib-CSim.jl"))
global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP
if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

global rob=CreateRobotKukaLwr();
Pos_init = [-0.3668, -0.0379, 0.8634]
Pos_ref =  [0.0, -0.0379, 0.5]

println("Mouvement pos initiale...")
θ_init=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868]
init_pos()

T = MGD(θ_init, rob)
print("Position initiale = ", T[1:3, 4]) # Verifie comme etant la bonne Pos_init.

# Cherche de la position articulaire de reference
println("\n1. Optimisation pour la pos ref...")
function f(θ)
    T = MGD(θ, rob)
    Pos_actuel = T[1:3, 4] # 1x3 (vecteur position qui vient de la matr de Transform T)
    return norm(Pos_ref - Pos_actuel)^2
end

@time results=optimize(f, θ_init, LBFGS())
theta=Optim.minimizer(results)

println("2. Valeur trouvé!\nMouvement vers le Pos_ref...")
controlrobot(theta, clientID, simx_opmode_oneshot, simx_opmode_oneshot, 7, objectname_kuka)
sleep(2)

T = MGD(theta, rob)
println("3. Validation!\npos ref théorique = ", T[1:3,4])
@printf "ecart theorique (Pos_ref - Pos_optm) = %.2f" (Pos_ref - T[1:3,4])
θtemp = getjointposition(clientID, 7, simx_opmode_oneshot, objectname_kuka)
T = MGD(θtemp, rob)
println("pos ref pratique = ", T[1:3,4])
@printf "ecart theorique (Pos_ref - Pos_coppelia) = %.2f" (Pos_ref - T[1:3,4])

stopsimulation(clientID, simx_opmode_oneshot)
