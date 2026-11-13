using JuMP
using Ipopt
using Plots, Polynomials
using Optim

# connexion avec VREP
const current_dir = @__DIR__
PATHCSIM = joinpath(current_dir, "lib")
include(joinpath(PATHCSIM, "lib-robotique.jl"))
include(joinpath(PATHCSIM, "Lib-CSim.jl"))
global clientID=startsimulation(simx_opmode_oneshot)
if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

# robot kuka
global rob=CreateRobotKukaLwr();
Pos_init = [-0.3668, -0.0379, 0.8634]
Pos_ref =  [0.15, -0.0379, 0.5]

# mise en pos initiale
println("Mouvement pos initiale...")
θ_init=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868]
init_pos()
T = MGD(θ_init, rob)
println("Pos initiale : ", round.(T[1:3,4], digits=4))

# obstacles
circle1 = [-0.15, 0.5]
circle2 = [0.0, 0.0]
r1 = 0.15
r2 = 0.2

function trajectory_optimization(pa, pb, circle_center_1, circle_center_2, R1, R2)
    # Définir les points de depart et d'arrivé
    xₐ, zₐ = pa
    xᵦ, zᵦ = pb
    ox1, oz1 = circle_center_1
    ox2, oz2 = circle_center_2

    # Modle d'optimisation
    model = Model(Ipopt.Optimizer)

    # Variables : 6 coeff pour avoir une puissance de x^5
    @variable(model, c[1:6])

    # Fonction objectif : minimiser la courbure de la trajectoire
    # Exemple : Minimiser la derivee seconde (courbure)
    @objective(model, Min, c[3]^2 + c[4]^2 + c[5]^2 + c[6]^2)

    #Contraintes : passage par les points de depart et d'arrivée
    @constraint(model, c[1] + c[2]*xₐ + c[3]*xₐ^2 + c[4]*xₐ^3 + c[5]*xₐ^4 + c[6]*xₐ^5 == zₐ)
    @constraint(model, c[1] + c[2]*xᵦ + c[3]*xᵦ^2 + c[4]*xᵦ^3 + c[5]*xᵦ^4 + c[6]*xᵦ^5 == zᵦ)

    #Contraintes pour eviter le cercle
    x_samples = range(xₐ, xᵦ, length=50)
    for x in x_samples
        z = c[1] + c[2]*x + c[3]*x^2 + c[4]*x^3 + c[5]*x^4 + c[6]*x^5
        @NLconstraint(model, (x - ox1)^2 + (z - oz1)^2 ≥ R1^2)
        @NLconstraint(model, (x - ox2)^2 + (z - oz2)^2 ≥ R2^2)
    end

    # Resoudre le probleme d'optimisation
    optimize!(model)

    # Recuperer les coefficients optimaux
    c_opt = value.(c)
    return c_opt
end

# appel à optimisation
pa = (Pos_init[1], Pos_init[3])
pb = (Pos_ref[1], Pos_ref[3])
c_opt = trajectory_optimization(pa, pb, circle1, circle2, r1, r2)
p = Polynomial(c_opt)
println(p)

## PLOT pour la question 3.b
# xₐ = pa[1]
# xᵦ = pb[1]
# fig = plot(p, xₐ, xᵦ, label="traj", lw=3, color=:blue, 
#            xlims=(-0.5, 0.3), ylims=(-0.2, 1.0), xlabel="x (m)", ylabel="z (m)")
# theta_vals = range(0, 2π, length=100)
# # obstacel 1
# plot!(fig, circle1[1] .+ r1*cos.(theta_vals), circle1[2] .+ r1*sin.(theta_vals), 
#       label="cercle 1", color=:red, fill=(true, :orange, 0.3))
# # osbtacle 2
# plot!(fig, circle2[1] .+ r2*cos.(theta_vals), circle2[2] .+ r2*sin.(theta_vals), 
#       label="cercle 2", color=:red, fill=(true, :red, 0.3))
# display(fig)

## Commande polynomiale articulaire pour coppelia
x_traj = range(pa[1], pb[1], length=50)
global θ_current = θ_init

println("Mouvevemnt de la trajectoire...")
for k in x_traj
    zₖ = c_opt[1] + c_opt[2]*k + c_opt[3]*k^2 + c_opt[4]*k^3 + c_opt[5]*k^4 + c_opt[6]*k^5
    Pos = [k, -0.0379, zₖ]

    function f(q)
        T = MGD(q, rob)
        Pos_actuel = T[1:3, 4] 
        return norm(Pos - Pos_actuel)^2
    end

    results = optimize(f, θ_current, LBFGS()) 
    global θ_current = Optim.minimizer(results)

    controlrobot(θ_current, clientID, simx_opmode_oneshot, simx_opmode_oneshot, 7, objectname_kuka)
end

T = MGD(θ_current, rob)
println("Pos Finale : ", round.(T[1:3,4], digits=4))

stopsimulation(clientID, simx_opmode_oneshot)