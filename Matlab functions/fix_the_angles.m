function  [q11,q22,q33,q44] = fix_the_angles(q)

time=(0:0.2:10)';
q11=[time,q(1:51,1)];
q22=[time,q(1:51,2)];
q33=[time,q(1:51,3)];
q44=[time,q(1:51,4)];
assignin('base', 'q11', q11);
assignin('base', 'q22', q22);
assignin('base', 'q33', q33);
assignin('base', 'q44', q44);

%convert to degs
q11_deg=rad2deg(q11(:,2))
q22_deg=rad2deg(q22(:,2))
q33_deg=rad2deg(q33(:,2))
q44_deg=rad2deg(q44(:,2))

end