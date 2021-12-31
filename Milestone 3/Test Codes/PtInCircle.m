function InCircle = PtInCircle(Xpt,Xcenter,r)
if (((Xpt(1)-Xcenter(1))^2)+((Xpt(2)-Xcenter(2))^2)<=r^2)
    InCircle = true;
else
    InCircle = false;
end