function S = skew(w)
%SKEW Return the 3-by-3 skew-symmetric matrix of a vector.
w = w(:);
S = [0, -w(3), w(2);
     w(3), 0, -w(1);
     -w(2), w(1), 0];
end
