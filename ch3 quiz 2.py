from modern_robotics import TransInv
from modern_robotics import VecTose3
from modern_robotics import MatrixExp6
from modern_robotics import ScrewToAxis
from modern_robotics import MatrixLog6
from modern_robotics import Adjoint
from modern_robotics import TransInv
import numpy

print('question 7')
Tsa = [[0,-1,0,0],[0,0,-1,0],[1,0,0,1],[0,0,0,1]]
Vs = [3,2,1,-1,-2,-3]
adjoint = Adjoint(TransInv(Tsa))
print('Va')
print(numpy.matmul(adjoint,Vs))

print('question10')
print('Adjoint Tsb = ')
Fb = [1,0,0,2,1,0]
adjoint = Adjoint(TransInv([[1,0,0,0],[0,0,1,2],[0,-1,0,0],[0,0,0,1]]))
print(adjoint)
print('Fa=')
adjoint = numpy.array(adjoint).transpose()
print(numpy.matmul(adjoint,Fb))

print('question 11: ')
print(TransInv([[0,-1,0,3],[1,0,0,0],[0,0,1,1],0,0,0,1]))

print('question 12: ')
print(VecTose3([1,0,0,0,2,3]))

print('question 13:')
print(ScrewToAxis([0,0,2],[1,0,0],1))

print('Question 14: ')
print(MatrixExp6([[0,-1.5708,0,2.3562],[1.5708,0,0,-2.3562],[0,0,0,1],[0,0,0,0]]))

print('Question 15')
print(MatrixLog6([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]]))