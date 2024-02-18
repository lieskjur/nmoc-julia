using JuMP
using GLPK

A = [
    -1 1
    1 6
    4 -1
]
c = [1, 1]
b = [1, 15, 10]

# Primal problem
primal = Model(GLPK.Optimizer)

@variable(primal, x[1:2] >= 0)
@constraint(primal, A * x <= b)
@objective(primal, Max, c'x)

JuMP.optimize!(primal)

println("x = ", value.(x))
println("c'x = ", objective_value(primal))

# Dual problem
dual = Model(GLPK.Optimizer)

@variable(dual, y[1:3] >= 0)
@constraint(dual, A' * y >= c)
@objective(dual, Min, b'y)

JuMP.optimize!(dual)

println("y = ", value.(y))
println("b'y = ", objective_value(dual))
