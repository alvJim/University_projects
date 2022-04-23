# Aerodynamics 02

In this paper an aerodynamic study of the CRM in transonic flow will be carried out. 
For this purpose, a brief development of the numerical methods used and an unstructured mesh 
for the model to be studied, both in 2D and 3D, will be carried out.

Subsequently, the validation of the methodology used and the obtaining of the pressure distribution 
for different sections of the wing will be carried out. All this will be compared with reference results.

Finally, we have experimented with the CRM, both in 2D and 3D, for two cases, viscous and non-viscous. 
In the 2D simulations, in both cases, relevant parameters such as Mach number and angle of attack were 
changed in order to see the behavior of the studied airfoil. On the other hand, the 3D simulations 
have not been performed in viscous flow due to the high computational cost, and therefore, it has been performed in non-viscous.

This work has been carried out with three programs. 
The first and most relevant was SU2, which is in charge of the physical simulation. 
This program has no user interface so we have always worked from the Visual Studio Code program and from the computer terminal. 
Referring to the programming part, the way of working was looking at the different codes uploaded to the SU2 web page, 
understanding them, making changes on them and extending them.

To be able to perform the simulation in SU2 it was necessary to have a mesh of the object for the study of the airfoils 
in 2D and a block for the study of the airplane in 3D. All the meshes have been made with the Pointwise program.

Finally, the ParaView program was used to visualize the results.

You can find different folders with the 2D results for both viscous and non-viscous cases and for the 3D cases. 
It is also possible to find a folder with the codes used in SU2.

Since there are many folders and different programs have been used, it is recommended to read the documentation.
