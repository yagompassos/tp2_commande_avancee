using Distributed 
addprocs(4) # On ajoute 4 coeurs de processeur
#----------------
@everywhere global Te1 = 0.4; #sampling rate of task test1
@everywhere global Te2 = 0.3; #sampling rate of task test2

@everywhere function test1(::Timer)
    sleep(Te1)
    println("Te1=", Te1)
    println("------")
end

@everywhere function test2(::Timer)
    sleep(Te2)
    println("Te2=", Te2)
end

@everywhere function test4()
    global t2 = Timer(test2, 0, interval = Te2)
    wait(t2)
    return t2
end

@everywhere function test3()
    global t1 = Timer(test1, 0, interval = Te1)
    wait(t1)
    return t1
end

@everywhere function test5()
    close(t1)
end

@everywhere function test6()
    close(t2)
end

t1 = remotecall_fetch(test3, 4); # execute the function test3 which launchs task test2 on proc 4
t2 = remotecall_fetch(test4, 3); #execute the function test4 which launchs task test1 on proc 3

sleep(10)

remotecall_fetch(test5, 4); # close task test1
remotecall_fetch(test6, 3); # close task test2
