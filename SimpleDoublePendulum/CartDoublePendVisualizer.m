classdef CartDoublePendVisualizer < Visualizer

  methods
    function obj = CartDoublePendVisualizer(plant)
      % Construct visualizer, and set l1 and l2 parameters
      
      typecheck(plant,'CartDoublePendPlant');
      
      obj = obj@Visualizer(plant.getStateFrame());
      
      obj.l1 = plant.l1;
      obj.l2 = plant.l2;
      obj.m1 = plant.m1;
      obj.m2 = plant.m2;
    end
    
    function draw(obj,t,x)
      l1=obj.l1; l2=obj.l2; m1=obj.m1; m2=obj.m2;
      scale=0.1;
      
      x1 = l1*[sin(x(1)); -cos(x(1))];
      x2 = x1 + l2*[sin(x(1)+x(2)); -cos(x(1)+x(2))];
      cart = x(3);
      line([cart, cart+x1(1)], [0, x1(2)],'LineWidth',2.0);
      line([cart+x1(1), cart+x2(1)], [x1(2), x2(2)],'LineWidth',2.0);

      th = 0:0.1:2*pi;
      fill(cart + x1(1) + scale*m1*sin(th), x1(2) + scale*m1*cos(th),[0 0 1]);
      fill(cart + x2(1) + scale*m2*sin(th), x2(2) + scale*m2*cos(th),[0 0 1]);
      theta = pi*[0:0.05:2];

      wb = .3; hb=.15;
      wheelr = 0.05;
      adj = 0.1;
      lwheel = [-wb/2 + wheelr*cos(theta); -hb-wheelr + wheelr*sin(theta)]';
      base = [wb*[1 -1 -1 1]; hb*[1 1 -1 -1]]';
      patch(cart+base(:,1), -adj+base(:,2),0*base(:,1),'b','FaceColor',[.3 .6 .4])
      patch(cart+lwheel(:,1), -adj+lwheel(:,2), 0*lwheel(:,1),'k');
      patch(cart+wb+lwheel(:,1), -adj+lwheel(:,2), 0*lwheel(:,1),'k');
      
      l = 1.2*(l1+l2);
      axis equal;
      axis([-l l -l l])

      title(['t = ', num2str(t,'%0.2f')]);
    end
  end

  properties
    l1=1;    % length of link 1
    l2=2;    % length of link 2
    m1=1;
    m2=1;
  end
  
end
