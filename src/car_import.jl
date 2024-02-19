using JuMP
using HiGHS

model = JuMP.Model(HiGHS.Optimizer)

@variable(model, x[1:2] >= 0)
@constraints(model, begin
    x[1] * 2.5 + x[2] * 0.5 <= 50
    x[1] + x[2] <= 60
end)
@objective(model, Max, x[1] * 0.25 + x[2] * 0.075)

JuMP.optimize!(model)

println("x = ", value.(x))
println("f(x) = ", objective_value(model))

