using JuMP
using OSQP
using LinearAlgebra
using Plots
using Distributions

# Discrete time state-space representation
dt = 1e-2
A = I(4) + [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0] * dt
B = [0 0; 0 0; 1 0; 0 1] * dt

# Optimal control problem
N = 100 # MPC horizon
M = 1000 # total number of steps

x0 = [2, 1, 0, 0] # initial state
u_max = 2 # symmetric upper and lower bound on inputs

Q = 1e0 * I(4)
R = 1e-2 * I(2)

# JuMP model of the MPC problem
model = JuMP.Model(OSQP.Optimizer)

## modified solver settings for increased accuracy
set_optimizer_attribute(model, "verbose", false)
set_optimizer_attribute(model, "polish", true)
set_optimizer_attribute(model, "eps_abs", 1e-4)
set_optimizer_attribute(model, "eps_rel", 1e-4)

## states and inputs as optimized variables
@variables(model, begin
    x[1:4, 1:N+1]
    -u_max <= u[1:2, 1:N] <= u_max
end)

## dynamics as a constraint between states and inputs
@constraint(model, [k = 1:N], x[:, k+1] == A * x[:, k] + B * u[:, k])

## standard quadratic objective function
@objective(model, Min,
    sum(x[:, k]' * Q * x[:, k] for k = 1:N+1) + sum(u[:, k]' * R * u[:, k] for k = 1:N)
)

# MPC simulation
## pre-allocation
xs = zeros(4, M + 1)
us = zeros(2, M)

## initial state
xs[:, 1] .= x0

## simulation loop
for i = 1:M
    fix.(x[:, 1], xs[:, i], force=true) # current state assigned as first MPC state
    JuMP.optimize!(model) # MPC optimization

    us[:, i] .= value.(u)[:, 1] # first MPC input assigned as current input
    noise = rand(Uniform(-1, 1), 2) # random input noise

    xs[:, i+1] .= A * xs[:, i] + B * (us[:, i] + noise) # next state 
end

# Visualization
plt = plot(layout=grid(2, 1, heights=[0.6, 0.4]))

for i = 1:4
    plot!(plt, xs[i, :], label="x$i", subplot=1)
end
for i = 1:2
    plot!(plt, us[i, :], label="u$i", subplot=2)
end

display(plt)
