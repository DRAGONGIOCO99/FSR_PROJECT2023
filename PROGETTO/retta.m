function P = retta(p1, p2)
    
    d=norm(p2-p1);
    s=linspace(0,1,ceil(200*d));
    P=(1-s).*p1'+s.*p2';
    P=P';

end

