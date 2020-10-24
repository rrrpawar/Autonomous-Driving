%This function is the multivariable Newtons and Gauss-Newtons methods for 
%solving the GPS equations.

%The inputs for the function are 
  %A, B, and C are the coordinates of the satellites
  %Guess is the first guess
  %t is the given time error
  %LA is the amount of satellites (equal to the length of the vector A)

function Result=MVNewtonsInputs(Guess,A,B,C,t,LA)

%Guess is transposed so it will work in the function in Newton's Method
Guess=Guess';



%c is light speed
c=299792.458;


%F is the function of the satellites positions in vector form.
%D is the Jacobian of F

%FCreator and DCreator are functions that create the vectors/matrices for F and
%D. They take the satellite coordinates, the time error, the amount of 
%satellites and the current guess as inputs. These functions exist so non linear
%systems of any amount of satellites can be created easily.

F=FCreator(Guess,LA,A,B,C,t);
D=DCreator(Guess,LA,A,B,C,t);

%Error and iteration are values that are used in Newton's method
%Error is preset to 1 so the method will iterate
%Iterations is set to 1 to count how many iterations it takes to get to the
%error tolerance
error=1;
Iterations=1;

%This while loop is Newton's/Gauss Newton method for more than 1 dimension
%It iterates through the Newtons/Gauss Newton method 20 times
while Iterations < 20
    %V is the solution to the Jacobian and the function at the first
    %current guess
    V=(D\(-F));
    %The next guess is the previous guess added to V
    Guess=(Guess+V);
    
    F=FCreator(Guess,LA,A,B,C,t);
    D=DCreator(Guess,LA,A,B,C,t);
    
    %This is the backward error, the distance between F evaluated at the
    %current guess and zero. This is the error the loop is controlled by
    error=max(abs(F));
    %Iterations increases the iterations counter
    Iterations=Iterations+1;
end
error;
Iterations;
%This outputs the values for x, y, z, and d with 6 correct digits
Result=Guess;