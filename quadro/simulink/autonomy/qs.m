function sortvec =qs(vec)
len=length(vec);
if len>1
    p=floor(len/2);
    vec(p);
    l=[];
    r=[];
    i=1;
    while   i<=len
        if vec(i) <= vec(p) & i~=p
            l=[l,vec(i)];
        elseif i~=p
            r=[r,vec(i)] 
        end;
        i=i+1;
    end;
    l=qs(l);
    r=qs(r);
    sortvec=[l,vec(p),r];
else
    sortvec=vec;
end