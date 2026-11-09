#! julia
# on peut faire des commentaires en julia de cette façon
# L'utilisation de packages se fait avec la commande using 

using Plots

# Création d'une fonction avec affichage

N = 300;
x = -1.5:0.01:1.49;
global y = zeros(N); # on déclare cette variable comme globale.

function test_exemple(N, x)
    for i = 1:300
        y[i] = 2 * log10(2 - x[i])
    end
    return y
end

y = test_exemple(N, x);

fig = plot(x, y);
display(fig);

