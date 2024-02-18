using Plots
using JuMP
using OSQP

# Visualization
plt = plot(xlabel="x₁", ylabel="x₂", aspect_ratio=:equal)
contour!(plt, -1.5:1e-2:1.5, -1:1e-2:1, (x, y) -> x^2 + y^2)
plot!(plt, [0, 1], [-1, 1], label="2x₁ - x₂ = 1")
display(plt)

# Problem definition
Q = [2 0; 0 2]
c = [0, 0]

A = [-2 1]
b = [-1]

# Solution
model = Model(OSQP.Optimizer)

@variable(model, x[1:2])
@constraint(model, A * x <= b)
@objective(model, Min, 0.5 * x' * Q * x + c' * x)

JuMP.optimize!(model)

# Visualization and printout
println("x = ", value.(x))
println("f(x) = ", objective_value(model))

scatter!([value.(x)[1]], [value.(x)[2]], label="optimum")
