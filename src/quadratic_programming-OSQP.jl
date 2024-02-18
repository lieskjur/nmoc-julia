using Plots
using OSQP
using SparseArrays

# Visualization
plt = plot(xlabel="x₁", ylabel="x₂", aspect_ratio=:equal)
contour!(plt, -1.5:1e-2:1.5, -1:1e-2:1, (x, y) -> x^2 + y^2)
plot!(plt, [0, 1], [-1, 1], label="2x₁ - x₂ = 1")
display(plt)

# Problem definition
P = sparse([2 0; 0 2])
q = [0.0, 0.0]

A = sparse([-2 1])
l = [-Inf]
u = [-1.0]

# Solution
model = OSQP.Model()
results = OSQP.Results()

OSQP.setup!(
    model;
    P=P, q=q,
    A=A, l=l, u=u,
    verbose=false
)

OSQP.solve!(model, results)

# Visualization and printout
println("x = ", results.x)
println("f(x) = ", results.info.obj_val)

scatter!([results.x[1]], [results.x[2]], label="optimum")
