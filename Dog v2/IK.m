function q = IK(x)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明

clc

format long ;

L1 = 86.0700;
L2 = 306.0000;
L3 = 341.0400;
x = p( 1, 1 )
y = p( 2, 1 )
z = p( 3, 1 )

% 求xita3

xita3a = 180 - acosd( ( L2^2 + L3^2 - ( y^2 + z^2 - L1^2 + x^2 ) ) / ( 2 * L2 * L3 ) ) 
xita3b = -180 + acosd( ( L2^2 + L3^2 - ( y^2 + z^2 - L1^2 + x^2 ) ) / ( 2 * L2 * L3 ) ) 

% ------------------------

% 求xita1

A = acosd( abs( z ) / hypot( y, z ) )
B = acosd( L1 / hypot( y, z ) )

if( y <= 0 && z > 0 )
    
        xita1a = A - B
        xita1b = A + B
     
end
     
if( y <= 0 && z <= 0 )
    
       
        xita1a = 180 - A - B
        xita1b = 180 - A + B
    
end

if( y > 0 && z > 0 )
    
        xita1a = A + B
        xita1b = -A + B
    
end

if( y > 0 && z <= 0 )
    
        xita1a = -180 + A - B
        xita1b = -180 + A + B
    
end


%--------------------------------
        
















end