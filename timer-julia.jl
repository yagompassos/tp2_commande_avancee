global temps = 0.0;
global dt = 0.1;

function Control_Loop_Once(::Timer)
    println("temps=", temps)
    println(" ")
    global temps = temps + dt
end

global tcontrol = Timer(Control_Loop_Once, 0, interval = dt)
wait(tcontrol)
sleep(10.0)
close(tcontrol)
