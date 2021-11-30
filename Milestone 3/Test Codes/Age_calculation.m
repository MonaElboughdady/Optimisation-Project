function GA = Age_calculation(Robot,GA)
   for i = 1:Robot.number
       for j = 1: GA.Pop_size 
          for c = 1: GA.Pop_size
                if( GA.Population{i}(:,:,j) == GA.Population_int{i}(:,:,c))
                    GA.Age(j) = GA.Age(j)+1;
                end
           end
       end
    end
    GA.Age = fix(GA.Age./Robot.number);
end