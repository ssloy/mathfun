function p = Bisection(fun, a, b, tolX)
% BISECTION - finds the point x where fun(x) crosses zero with the desired
% tolX. 

if fun(a)*fun(b)>0 % if fun(a) is of the same sign as fun(b)
    error('f(a) and f(b) are of the same sign.');
end

p=(a+b)/2;
err=p-a;

while abs(err)>tolX
    if fun(a)*fun(p) > 0 % if fun(p) is of the same sign as fun(a)
        a=p; % move toward b
    else % if fun(p) is of different sign than fun(a)
        b=p; % move toward a
    end
    p=(a+b)/2;
    err=b-p;
end

end