# MPC-Design-Tools
VirtualArena package for the design of MPC controllers with closed-loop guarantees.

## `terminalCostES`

Consider the dynamical system
 
                            \dot{x}=f(t,x,u)                           (1)
 
and a control law k_aux : R x R^n -> R^n that stabilizes the origin exponentially quickly, with Lyapunov Certificated of exponential stability `esLyapCertificate`  (see help ESLyapunovCertificate).
 
Moreover, let the stage cost function l: R x R^n x R^m -> R satisfies
 
        l(t,x,k_aux(t,x))<= \sum_{i=1}^{length(a)} a(i) ||x||^i
 
for a vector a = [a1,a2,...,am]. Then, along the closed-loop (1) with u = k_aux (t,x), the function 
 
        m(t,x) = @(t,x) terminalCostES(t,x,esLyapCertificate,a)
 
satisfies
    
                   \dot{m}(t,x) <= -l(t,x, k_aux(t,x)).

**Reference**: 

Alessandretti A., Aguiar P. A., Jones C.. _"On Convergence and Performance Certification of a Continuous-Time Economic Model Predictive Control Scheme with Time-Varying Performance Index."_ Automatica (in Press), 2016.