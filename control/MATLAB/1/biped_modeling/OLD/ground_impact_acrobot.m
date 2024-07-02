function [value,isterminal,direction] = ground_impact_acrobot(t,in2)
%GROUND_IMPACT_ACROBOT
%    [VALUE,ISTERMINAL,DIRECTION] = GROUND_IMPACT_ACROBOT(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    29-Feb-2024 16:04:52

q1 = in2(1,:);
q2 = in2(2,:);
value = sin(q1+q2)+sin(q1);
if nargout > 1
    isterminal = 1.0;
end
if nargout > 2
    direction = -1.0;
end
end
