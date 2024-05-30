Matlab Control Scripts

This repository contains MATLAB scripts for designing and optimizing fractional PID controllers, specifically for systems with significant delays. The scripts include functionalities for calculating performance indices like IAE (Integral of Absolute Error), ISE (Integral of Squared Error), and ITAE (Integral of Time-weighted Absolute Error), and use Differential Evolution algorithms for optimization.

Contents

iae_ise.m
Purpose: Provides functions to calculate IAE and ISE for fractional order PID controllers using Particle Swarm Optimization (PSO)

Key Functions:

calcularIAE(param, Ga_completa): Computes the Integral of Absolute Error.

calcularISE(param, Ga_completa): Computes the Integral of Squared Error.

calcularITAE(param, Ga_completa): Computes the Integral of Time-weighted Absolute Error.

iae_ise_atraso.m

Purpose: Extends the basic IAE and ISE calculations to systems with delays.

Key Functions: Similar to iae_ise.m but adapted for delayed systems.

iae_ise_de.m

Purpose: Implements Differential Evolution (DE) optimization to tune fractional PID controller parameters.

Key Functions:

diferentialEvolution(fitnessFunc, numGenerations, populationSize, CR, F): Performs the optimization using DE.

fitnessFunc(param, Ga_completa): Defines the cost function for optimization.

iae_ise_deatraso.m

Purpose: Combines DE optimization with delay compensation for tuning fractional PID controllers in systems with delays.

Key Functions: Combines the functionalities from iae_ise_atraso.m and iae_ise_de.m.
