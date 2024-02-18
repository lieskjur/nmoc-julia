using OSQP
using LinearAlgebra
using Plots
using SparseArrays
using Distributions

# Discrete time state-space representation
dt = 1e-2
A = I(4) + [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0] * dt
B = [0 0; 0 0; 1 0; 0 1] * dt

n = 4 # number of states
m = 2 # number of inputs

# Optimal control problem
N = 100 # MPC horizon
M = 1000 # total number of steps

x0 = [2, 1, 0, 0] # initial state
u_max = 2 # symmetric upper and lower bound on inputs

Q = 1e0 * I(4)
R = 1e-2 * I(2)

# QP model of the problem
## Objective matrix (P)
objective_matrix = spzeros(N * (m + n), N * (m + n))
objective_matrix[1:N*m, 1:N*m] .+= kron(I(N), R) # input x input
objective_matrix[N*m+1:end, N*m+1:end] .+= kron(I(N), Q) # state x state

## Objective vector (q)
objective_vector = zeros(N * m + N * n)

## Constraint matrix (A)
constraint_matrix = spzeros(N * (n + m), N * (n + m))
constraint_matrix[1:N*n, 1:N*m] .+= kron(I(N), B) # state-constraints x input
constraint_matrix[1:N*n, N*m+1:end] .+= kron(I(N), -I(n)) # state-constraints x future state
constraint_matrix[n+1:N*n, N*m+1:end-n] .+= kron(I(N-1), A) # state-constraint x current state
constraint_matrix[N*n+1:end, 1:N*m] .+= kron(I(N), I(m)) # input-constraints x input

## Lower bounds (l)
lower_bounds = zeros(N * (n + m))
lower_bounds[N*n+1:end] .= -u_max # input-constraints

## Upper bounds (u)
upper_bounds = zeros(N * (n + m))
upper_bounds[N*n+1:end] .= u_max # input-constraints

# OSQP model setup
model = OSQP.Model()
results = OSQP.Results()

OSQP.setup!(
    model;
    P=objective_matrix, q=objective_vector,
    A=constraint_matrix, l=lower_bounds, u=upper_bounds,
    verbose=false
)

# MPC simulation
## pre-allocation
xs = zeros(n, M + 1)
us = zeros(m, M)

## initial state
xs[:, 1] .= x0

## simulation loop
for i = 1:M
    # initial state condition
    lower_bounds[1:n] .= -A * xs[:, i]
    upper_bounds[1:n] .= -A * xs[:, i]
    OSQP.update!(model, l=lower_bounds, u=upper_bounds)

    # MPC calculation
    OSQP.solve!(model, results)

    # policy application
    us[:, i] = results.x[1:m] # first MPC input assigned as current input
    noise = rand(Uniform(-1, 1), 2) # random input noise
    xs[:, i+1] .= A * xs[:, i] + B * (us[:, i] + noise) # next state 
end

# Visualization
plt = plot(layout=grid(2, 1, heights=[0.6, 0.4]))

for i = 1:n
    plot!(plt, xs[i, :], label="x$i", subplot=1)
end
for i = 1:m
    plot!(plt, us[i, :], label="u$i", subplot=2)
end

display(plt)
