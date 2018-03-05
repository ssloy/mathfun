Ts=0.035*1e-3; %35 micro seconds - sample time
Td=0.003; % desired T for the closed loop
%%
load('new_motor_data.mat');
U=u_samples/255*24; % from samples to volts
I=(i_samples)*2.5/(255*.185); % from samples to ampers
%% divide by experiments
div_ind=find(diff(time)>0.01);
ExpN=numel(div_ind)+1; %number of experiments
exp=cell(ExpN,1); 
for i=1:ExpN,
    % left index
    if i==1,
        left=1;
    else
        left=div_ind(i-1)+1;
    end
    % right index
    if i<ExpN,
        right=div_ind(i);
    else
        right=numel(time);
    end
    
    %extract trial
    exp{i}.time=time(left:right);
    exp{i}.U=U(left:right);
    exp{i}.I=I(left:right);
    
    % interpolate (uniform)
    ut=(exp{i}.time(1):Ts:exp{i}.time(end))';
    uU=interp1(exp{i}.time,exp{i}.U,ut);
    uI=interp1(exp{i}.time,exp{i}.I,ut);
    
    % define iddata structure with the uniform (interpolated) data
    exp{i}.data=iddata(uI,uU,Ts);
end
%% Merge data
full_data=exp{1}.data;
for i=2:ExpN,
    full_data=merge(full_data,exp{i}.data);
end
%% Identify
plant=tf(tfest(full_data,1,0));
tau=1/plant.den{1}(2);
K=plant.num{1}(2)/plant.den{1}(2);
%% compute PI
Kp=tau/K/T;
Ki=1/K/T;