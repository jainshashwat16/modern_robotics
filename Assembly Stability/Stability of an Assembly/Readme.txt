This program determines if friction is sufficient to hold a multibody assembly together. 
The program requires numpy and scipy to run.
The program was run on a python 3.7 interpreter, but it is expected that it is compatible with earlier versions of python 3 as well

This program was run on two different assemblies, first the example from the wiki. 
The files related to this assembly are contained in the "test assembly" folder

The second assembly was based off of the Three body arch from the book. The normal directions of the contacts with body 3 
were assumed to be 45 degrees, however this does not exactly match the angles shown in the diagram. 

To use the program two files are needed. 
Firstly, a body file containing the information in about each body with the following columns:
mass, x coordinate of cg, y coordinate of cg
Secondly a contacts file containing information about the contacts between each body should be included. It should 
have the following columns:
contact x coordinate, contact y coordinate, coefficient of friction, first body number, second body number, contact normal
direction relative to the first body in radians.
Note, the ground should be assigned the body number "0"