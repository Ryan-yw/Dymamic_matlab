clc

format long ;

L1 = 86.0700;
L2 = 306.0000;
L3 = 341.0400;
x = p( 1, 1 );
y = p( 2, 1 );
z = p( 3, 1 );

xita1 = xita1a;
xita3 = xita3a;



% ----------判断xita2坐标轴----------

% 参数

L = sqrt( y^2 + z^2 - L1^2 );

if(xita1 == xita1a)
    y = -L;
end
if(xita1 == xita1b)
    y = L;
end

% y
% 
% if( xita1 >= 0 && xita1 < 90 )
%     
%     y0 = -tan( xita1 ) * z
%     
%     if( y >= y0 )
%         
%         y = L;
%         
%     end
%     if( y < y0 )
%         
%         y = -L
%         
%     end
%     
% end
% 
% if( xita1 == 90 )
%     
%     if( y <= 0 )
%         
%         y = -L;
%         
%     end
%     if( y > 0 )
%         
%         y = L;
%         
%     end
%     
% end
% 
% if( xita1 > 90 && xita1 <= 180 )
%     
%     y0 = -tan( xita1 ) * z;
%     
%     if( y >= y0 )
%         
%         y = -L;
%         
%     end
%     if( y < y0 )
%         
%         y = L;
%         
%     end
%     
% end
% 
% if( xita1 > -90 && xita1 < 0 )
%     
%     y0 = tan( xita1 ) * z;
%     
%     if( y >= y0 )
%         
%         y = L;
%         
%     end
%     if( y < y0 )
%         
%         y = -L;
%         
%     end
%     
% end
% 
% if( xita1 == -90 )
%     
%     if( z <= 0 )
%         
%         y = L;
%         
%     end
%     if( Z > 0 )
%         
%         y = -L
%         
%     end
%     
% end
% 
% if( xita1 >= -180 && xita1 < -90 )
%     
%     y0 = tan( xita1 ) * z;
%     
%     if( y >= y0 )
%         
%         y = -L;
%         
%     end
%     if( y < y0 )
%         
%         y = L;
%         
%     end
%     
% end

% -----------------------------

%求xita2

A = acosd( abs( x ) / hypot( x, y ) )
B = acosd( ( L2^2 + x^2 + y^2 -L3^2 ) / ( 2 * L2 * hypot( x, y ) ) ) ;
B = real( B )


if( x <= 0 && y < 0 )
    
    if( xita3 >= 0 )
        
        xita2 = A - B
        
    end
    if( xita3 < 0 )
        
        xita2 = A + B
        
    end
    
end

if( x > 0 && y <= 0 )
    
    if( xita3 >= 0 )
       
        xita2 = 180 - A - B
        
    end
    if( xita3 < 0 )
        
        xita2 = 180 - A + B
        
    end
    
end

if( x <= 0 && y > 0 )
    
    if( xita3 >= 0 )
        
        xita2 = -A - B
        
    end
    if( xita3 < 0 )
        
        xita2 = -A + B
        
    end
    
end


if( x > 0 && y >= 0 )
    
    if( xita3 >= 0 )
        
        xita2 = -180 + A - B
        
    end
    if( xita3 < 0 )
        
        xita2 = -180 + A + B
        
    end
    
end


% ------------------------------------

















