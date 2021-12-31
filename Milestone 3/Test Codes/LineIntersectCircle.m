function intersects = LineIntersectCircle(X1,X2,Xcenter,r)
slope=(X1(2)-X2(2))/(X1(1)-X2(1));
const = X1(2)-slope*X1(1);
syms x y
eqn1 = y == slope*x+const;
eqn2 = (x - Xcenter(1))^2 + (y - Xcenter(2))^2 == r^2;
PtOfIntersection = solve({eqn1 eqn2},[x y]);
if(X1(1)>X2(1))
    temp = X2(1);
    X2(1) = X1(1);
    X1(1) = temp;
end
if (imag(PtOfIntersection.x) == 0) 
    if((PtOfIntersection.x(1)>=X1(1)&&PtOfIntersection.x(1)<=X2(1))||(PtOfIntersection.x(2)>=X1(1)&&PtOfIntersection.x(2)<=X2(1)))
        intersects = true;
    else
        intersects = false;
    end    
else
    intersects = false;
end
end