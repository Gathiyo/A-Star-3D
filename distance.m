function dist = distance(x1,y1,z1,x2,y2,z2)
%This function calculates the distance between any two cartesian 
%coordinates.
%   Copyright 2009-2010 The MathWorks, Inc.
dist=sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
