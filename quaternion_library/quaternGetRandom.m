function ab = quaternGetRandom()
   phi=rand()*2*pi;
   vec=rand(1,3);
   vec=vec/norm(vec);
   ab =[cos(phi/2) vec*sin(phi/2)];
end

