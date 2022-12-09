%%%%%%%%%%%% Copyright (C) 2021 Ajith Anil Meera  %%%%%%%%%%%%%%%%%%%%%
%
% The code accompanying the paper "Anil Meera, Ajith, and Martijn Wisse. 
% "Dynamic expectation maximization algorithm for estimation of linear systems 
%  with colored noise." Entropy 23.10 (2021): 1306.
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; 
% close all;
rand_w = []; rand_z = []; found_w = []; found_z = [];

% figure(10); plot([-2 2],[-2 2],'b-','linewidth',2); hold on
% err_full_s = zeros(100,14);     err_full_b = err_full_s;

SSE = [];  SSE_o = []; % To store SSE for parameter estimation 
for s_i = 1
    
SSE_theta = [];

theta_convg_1 = [];  theta_convg_2 = []; theta_convg_3 = [];
errP_Pp = [];
for expt = 1  
    
    
% expe = 4; % use for loading data, exp = 4 gives the data for standing still
% subexp = 2; % we did 2 experiments for this, pick 1 or 2 
% 
% load(char("C:\Users\LocalAdmin\Desktop\BEP_data\MATfiles\matdata"+expe+"."+subexp+".mat"));% load matfiles
% s_est = estimate_smoothness(Y(4000:4150)',0:.1:15);
% UIO_estimator;

if_dataset = 0;
if if_dataset==1
    if 0
        [Tt,model.real_cause,model.process_x,model.process_y,ideal_y,current,...
            voltage,model.sam_time,model.A,model.B,model.C] = DC_motor_data;
        model.ideal_x = model.process_x;
        model.t_end = Tt(end) + model.sam_time;
        model.t = Tt + model.sam_time;        
    else
%         [Tt,model,brain] = figure_8_drone_data;      % figure 8 drone data
        [model.t,model.real_cause,model.process_x,model.process_y,ideal_y,...
            model.sam_time,model.A,model.B,model.C,full_data] = ardrone_data;
        model.ideal_x = model.process_x;
        
        %%%%%%%%%%% REMOVE lATER
%         model.real_cause = smoothdata(model.real_cause','gaussian',20)';

    % Input normalization
        input_norm = (max(max(model.real_cause))-min(min(model.real_cause)));
        full_data.u = (full_data.u-mean(mean(model.real_cause)))./input_norm;
        model.B = model.B*input_norm;
        model.real_cause = (model.real_cause-mean(mean(model.real_cause)))...
                ./input_norm;
%         
        % Scaling down the inputs 
%         full_data.u = full_data.u/200;
%         model.B = model.B*200;
%         model.real_cause = model.real_cause/200;
        
%         model.B = model.B*100;
        %%%%%%%%%%%%%%%%%%%%%%%%
        
        model.t_end = model.t(end);
    end
    brain.ny = size(model.C,1);
else
    model.sam_time = .1;
    model.t_end = 32;
    Tt  = 0:model.sam_time:model.t_end-model.sam_time;
    model.t = Tt + model.sam_time;
        
%     model.real_cause(1,:) = exp(-.25*(model.t-12).^2);
%     model.real_cause(2,:) = exp(-.25*(model.t-20).^2);
%     model.real_cause(3,:) = .5*exp(-.25*(model.t-22).^2);
%     model.real_cause(4,:) = .25*exp(-.25*(model.t-20).^2);
%     model.real_cause(5,:) = -.25*exp(-.25*(model.t-25).^2);
%     model.real_cause(6,:) = -.5*exp(-.25*(model.t-18).^2);
    
%     for i1 = 1:5
%         model.real_cause(i1,:) = exp(-.25*sin(i1)*(model.t-16).^2);
%     end
    model.real_cause = exp(-.25*(model.t-12).^2);
%         model.real_cause(1,:) = .4*sin(.2*model.t) ; 
%         model.real_cause(2,:) = .4*cos(.4*model.t) ; 
%             model.real_cause(1,1:7) = zeros(1,7); model.real_cause(1,26:32) = zeros(1,7);
%         model.real_cause = (1/model.t_end)*(model.t);

%     
%     k =.8; m = 1.4; b = .4;  % b_critical = 2 sqrt(k*m)
%     model.A = [0 1; -k/m -b/m];
%     model.B = [0; 1/m];
%     model.C = [1 0];
    
    % Random spring damper systems
%     while 1
%         rng('shuffle')
%         model.A = [0 1; -rand(1) -rand(1)];
%         model.B = [0; rand(1)];
%         model.C = [1 0];
%         if isstable(ss(model.A,model.B,model.C,0))
%             break;
%         end
%     end
    
%     model.A = [-.25 1; -.5 -.25];
%     model.B = [1; 0];
%     model.C = [.125 .1633; .125 .0676; .125 -.0676; .125 -.1633];

      % A random sampled model
%       model.A = [-0.7088 -0.3845; -0.1322 -0.6666]; model.B = [0.5881; 0.6280];
%       model.C = [0.8545 0.9530; 0.1434 -0.1078; -0.2447 0.7440; -0.6219 -0.1403];
       
%        model.A = [-0.6205 0.9114;  -0.8175  0.0238]; model.B = [0.1710; -0.4317];
%        model.C = [0.5275 -0.9547; 0.6429 -0.5260; 0.9780 -0.5931; 0.9366 0.5134];
  
      % Randomly selected model matrices
%     while 1
%         rng('shuffle')
%         brain.ny = 4; brain.nx = 2;     % drone 9,2
%         brain.nu = size(model.real_cause,1);
%         model.A = -1+2*rand(brain.nx); 
%         model.B = -1+2*rand(brain.nx,brain.nu);
%         model.C = -1+2*rand(brain.ny,brain.nx);
%         if isstable(ss(model.A,model.B,model.C,0))
%             break;
%         end
%     end
    
%        model.A = -2; model.B = 2; model.C = 1; 
    
        %%% System used in DEM Entropy paper
        model.A = [ 0.0484    0.7535;  -0.7617   -0.2187];
        model.B = [0.3604;     0.0776];
        model.C = [0.2265 -0.4786; 0.4066 -0.2641; 0.3871 0.3817; -0.1630 -0.9290];
        
%     model.A = -.5;   model.B = .5;    model.C = .5;

%     model.A = -1; model.B = 1; model.C = -1;    
%     brain.A = 0; brain.B = 1; brain.C = -1; 

end

% Spring damper - fixed prior parameters
% brain.A = [0 1; -2 -1];
% brain.B = [0; 2];
% brain.C = [1 0];
          
% brain.A = model.A; brain.B = model.B; brain.C = model.C;
while 1
%     rng('shuffle');
    rng(33);        % rng(33) for drone data in paper, also for frontiers spring 
    brain.A = -2+4*rand(size(model.A));    %brain.A(1,1) = 0; brain.A(2,1) = 1;
    brain.B = -2+4*rand(size(model.B));
    brain.C = -2+4*rand(size(model.C));
    
%     brain.A = -2+4*rand(3,3); brain.B = -2+4*rand(3,1); brain.C = -2+4*rand(1,3); 
    
%     eig(brain.A)
    % Initialize known parameters
%     brain.A = model.A; brain.B = model.B;
    
%     brain.A(1,1:2) = model.A(1,1:2); brain.B(1,1) = model.B(1,1); % spring damper
%     brain.C = model.C;  
        
    if isstable(ss(brain.A,brain.B,brain.C,0))
        break;
    end
end
% brain.A(2,1) = -.8; brain.A(2,2) = -.5; brain.B(2,1) = .5; 

brain.C = model.C; 
% brain.B(1,:) = model.B(1,:); 

% Sample priors from a circle
% rng('shuffle'); rand11 = rand(1);
% brain.A = (5*cos(2*pi*rand11)); 
% brain.B = 5*sin(2*pi*rand11); 
% brain.C = model.C;

% brain.A = model.A;  brain.C = model.C; % brain.B = model.B;
% brain.A = model.A + brain.A; brain.B = model.B + brain.B; 
% brain.C = model.C + brain.C;
% brain.A = 10^4*brain.A; brain.B = 10^4*brain.B; brain.C = 10*brain.C;

% A randomly sampled prior model
% brain.A = [-0.1551 0.1548; 1.4687 -1.8506]; brain.B = [0.5671; -0.7186];
% brain.C = [1.5130 1.9789; -1.1115 1.7768; 1.6692 -0.0220; 1.2345 -0.2791];


% motor data grey box
% brain.C = model.C; brain.B(1:2,1) = [0; 0]; %brain.A(3,3) = model.A(3,3);
% brain.A(1:3,1) = [0;0;0]; brain.A(1,2:3) = [1 0];

% brain.A = 1+0*model.A; brain.B = -1+0*model.B; brain.C = -1+0*model.C; 

%     brain.A = -.5+1*rand(size(model.A));    %brain.A(1,1) = 0; brain.A(2,1) = 1;
%     brain.B = -.5+1*rand(size(model.B));
%     brain.C = -.5+1*rand(size(model.C));

% model.real_cause = model.real_cause;
brain.prior_cause = model.real_cause;%  model.real_cause; % ;

brain.prior.theta = [reshape(brain.A',[],1); reshape(brain.B',[],1); ...
    reshape(brain.C',[],1)];
model.theta = [reshape(model.A',[],1); reshape(model.B',[],1); ...
    reshape(model.C',[],1)];

% Prior covariance of parameters
brain.prior.Pp = exp(32)*eye(size(brain.prior.theta,1)); % exp(4) latest
%     brain.prior.Pp = diag(exp(1./[-1.5 -0.01 -0.7 -1.5...
%         -0.01 -20 -1.6094 -1.6094 -1.6094 -3 -1.6094 -3 -1.6094 -1.6094]).^2);

%       brain.prior.Pp(5:12,5:12) = exp(0)*eye(8); %brain.prior.Pp(24,24) = exp(32);
%     brain.prior.Pp(3:end,3:end) = exp(15)*eye(4);
%     brain.prior.Pp(13:16,13:16) = exp(32)*eye(4);
    brain.prior.Pp(1:6,1:6) = exp(6)*eye(6);
%     brain.prior.Pp(3,3) = exp(32); %brain.prior.Pp(3,3) = exp(32);

% Motor data grey box
% brain.prior.Pp(1:7,1:7) = exp(32)*eye(7); 
% brain.prior.Pp(7,7) = exp(32); 
% brain.prior.Pp(10:11,10:11) = exp(32)*eye(2);
% brain.prior.Pp(13:15,13:15) = exp(32)*eye(3);

% spring damper system
% brain.prior.Pp(1:2,1:2) = exp(15)*eye(2); brain.prior.Pp(5,5) = exp(15)*eye(1);
% brain.prior.Pp(7:8,7:8) = exp(15)*eye(2);

% Matrices for M step; Set hyperpriors
brain.prior.h = [0; 0 ];%[20; 3];
brain.h = brain.prior.h;
brain.prior.Ph = exp(3)*eye(2);   % exp(-32) working one
% brain.prior.Ph = diag([exp(25),exp(20)]);

brain.Pp = brain.prior.Pp;
brain.Ph = brain.prior.Ph;
brain.theta = brain.prior.theta;
%     brain.S_theta = sqrt(diag(pinv(brain.Pp))); % cross check this

brain.nt = size(model.t,2);
if_predict_y = 0;
if_UIO = 0;
UIO_gamma = 125;%3050; %motor
% UIO_gains = [1200 1200 1200 1400 1500];

if_cause = 0;   % should model with known causes?
if_MF_X = 1;    % Mean field terms for states
if_MF_p = 1;    % Mean field terms for parameters
if_plot_FE_manifold = 0;   % If free energy manifold should be plotted for visualization

iter = 1;
% s_est = zeros(1,iter);
% s_realA = zeros(1,iter);

model.s =  .5;%model.sam_time;%.2*s_i-.1; % 0.0086 = ardrone wind %0.01 + exp(-16); 
brain.s =  model.s;%0.01 + exp(-16);    % motor 

model.p = 6;
model.d = 2;
brain.p = 6; % motor data 5 % figure_8 2 % ardrone 1
brain.d = 2; % motor data 2 % figure_8 1 % ardrone 0

model.sigma_w = exp(-4);%     %motor data exp(-6)  figure_8 exp(-8)
model.sigma_z = exp(-4);%exp(-(5-(7/14)*(expt-1)));%  %motor data exp(-13)    figure_8 exp(-4)
brain.sigma_v = exp(-16); %                  % figure_8 exp(-8)  % -.6 for input estimation
brain.sigma_w = exp(0);%model.sigma_w;
brain.sigma_z = exp(0);%model.sigma_z;%exp(-4);

brain = init_brain(brain);
model = generative_process(model,brain,if_dataset);

% figure; imagesc(model.process_y');colorbar
%%
brain.Y_embed = generalized_process(model.process_y,brain.prior_cause,...
    model.t,model.sam_time,brain.ny,brain.nu,brain.p,brain.d);

if if_plot_FE_manifold ==1
    figure(342); clf; figure(343); clf;
end

for para = 1:20
    
%     if mod(para,5) == 0
%         while 1
%             rng('shuffle');
%             %     rng(2);
%             brain.A = -1+2*rand(size(model.A));    %brain.A(1,1) = 0; brain.A(2,1) = 1;
%             brain.B = -1+2*rand(size(model.B));
%             brain.C = -1+2*rand(size(model.C));
%             if isstable(ss(brain.A,brain.B,brain.C,0))
%                 break;
%             end
%         end
%     end
    
    output = observer(model,brain,if_UIO,UIO_gamma,if_cause,if_dataset,...
                        if_predict_y,if_MF_X);
    if if_plot_FE_manifold == 1 && para == 15
        brain = plot_FE_manifold(output,brain,model,if_MF_p,para);        
    else
        brain = update_parameters(output.DEM_x,brain,if_MF_p);
        brain = set_brain(brain);
    end
    
    plot_estimation(model,brain,output,para);
    
    
    
    %%%% For drone dataset%%%%%%%%%%%%%%%%
    if if_dataset
        n_step = 150;                           % output.DEM_x(end,1:brain.nx)'
        y_pred.b = n_step_ahead_prediction(n_step,0*model.ideal_x(end,:)',...
            brain.A,brain.B,brain.C,model.sam_time,full_data.u(:,1:brain.nt+n_step),brain.nt);
        figure(122); clf;
        rectangle('Position',[model.t(end),-1,(50+n_step)*model.sam_time,2.5],...
            'FaceColor',[.9 .9 .9],'LineStyle','none'); hold on
        figa.a1 = plot(full_data.t(1:brain.nt+n_step),full_data.y(1:brain.nt+n_step,:),...
            'color','r','Linewidth',1); hold on;
        figa.a2 = plot(model.t,(brain.C*output.DEM_x(:,1:brain.nx)')',...
            'color','b','Linewidth',1); hold on;
        figa.a3 = plot((brain.nt+(0:n_step-1))*model.sam_time,y_pred.b,...
            'color','#77AC30','Linewidth',1); hold on;
        legend([figa.a1(1),figa.a2(1),figa.a3(1)],...
            {'Measured output','Prediction on training data',...
            'Prediction on test data'},'location','best');
        xlabel('Time(s)'); ylabel('y');
    end
    %%%%%%%%%%%%%%%%%%%%%%%$$$$$$$$$$$$$$$$$$
    
    
    
%     y_pred.m = n_step_ahead_prediction(n_step,model.ideal_x(end,:)',...
%         model.A,model.B,model.C,model.sam_time,full_data.u(:,1:brain.nt+n_step),brain.nt);
%     plot(brain.nt+(0:n_step-1),y_pred.m,'g-'); hold on;
    
                    
%     load ardrone2FlightData7_wind2_hPhiDot t ts uLin xLin yLin zPi wPi s A B C;
%     data_range = 1:400;
%     figure(112); clf; plot(t(data_range),yLin(:,data_range)','r'); hold on;
%     data_range = 250:400;
%     [yyy1,~] = lsim(ss(brain.A,brain.B,brain.C,[]),...
%         uLin(:,data_range),t(data_range),output.DEM_x(end,1:brain.nx));
%     plot(t(data_range), yyy1','b')
    
    % Compare the similarity transformed parameters of model and brain
%     sys_b = ss(brain.A,brain.B/input_norm,brain.C,[]);
%     [csys_b, TTb] = canon(sys_b,'companion');
%     csys_b = [reshape(csys_b.A',1,[]) reshape(csys_b.B',1,[]) reshape(csys_b.C',1,[])];
%     [csys_m, TTm] = canon(ss(model.A,model.B/input_norm,model.C,[]),'companion');
%     csys_m = [reshape(csys_m.A',1,[]) reshape(csys_m.B',1,[]) reshape(csys_m.C',1,[])];
%     figure(113); bar([csys_m' csys_b']);
    
    fprintf('Update %d\n',para);
end

%     figure(343); hold on; plot(brain.memory.theta(1,1:end-1),brain.FreeE,'r-.','Linewidth',1);
%     hold on; plot(brain.memory.theta(2,1:end-1),brain.FreeE,'b-.','Linewidth',1);


%     [SSE_theta(:,expt), para_bench] = SSE_parameter(brain,model,output); 
  
    %%%%%%%%%%%% Drone dataset %%%%%%%%%%%%%%%%%%%%%
    if if_dataset
        y_pred.p = n_step_ahead_prediction(n_step,0*model.ideal_x(end,:)',...
            para_bench.p.A,para_bench.p.B,para_bench.p.C,model.sam_time,full_data.u(:,1:brain.nt+n_step),brain.nt);
        figure(122);
        figa.a4 = plot((brain.nt+(0:n_step-1))*model.sam_time,y_pred.p,'m-',...
            'Linewidth',1); hold on;
        y_pred.s = n_step_ahead_prediction(n_step,0*model.ideal_x(end,:)',...
            para_bench.s.A,para_bench.s.B,para_bench.s.C,model.sam_time,full_data.u(:,1:brain.nt+n_step),brain.nt);
        figa.a5 = plot((brain.nt+(0:n_step-1))*model.sam_time,y_pred.s,'c-',...
            'Linewidth',1); hold on;
        y_pred.em = n_step_ahead_prediction(n_step,0*model.ideal_x(end,:)',...
            para_bench.em.A,para_bench.em.B,para_bench.em.C,model.sam_time,full_data.u,brain.nt);
        figa.a6 = plot((brain.nt+(0:n_step-1))*model.sam_time,y_pred.em,'y-',...
            'Linewidth',1); hold on;
        legend([figa.a1(1),figa.a2(1),figa.a3(1),figa.a4(1),figa.a5(1),figa.a6(1)],...
            {'Measured output','DEM on training data',...
            'DEM on test data','PEM','SS','EM'},'location','best');
        
        n_step_SSE = 1:n_step;%100*(1:3);
        SSE_y_pred = [sum((y_pred.b(:,n_step_SSE) - full_data.y(brain.nt+n_step_SSE,:)').^2);...
            sum((y_pred.p(:,n_step_SSE) - full_data.y(brain.nt+n_step_SSE,:)').^2);...
            sum((y_pred.s(:,n_step_SSE) - full_data.y(brain.nt+n_step_SSE,:)').^2);...
            sum((y_pred.em(:,n_step_SSE)- full_data.y(brain.nt+n_step_SSE,:)').^2)]';
        figure(66); plot(SSE_y_pred,'LineWidth',2);
        legend('DEM','PEM','SS','EM'); set(gca,'YScale','log','FontSize',14);
        xlabel('n steps');  ylabel('Squared prediction error');
        fprintf('MSPE [DEM,PEM,SS,EM]:%f,%f,%f,%f\n',mean(SSE_y_pred(:,1)),...
            mean(SSE_y_pred(:,2)),mean(SSE_y_pred(:,3)),mean(SSE_y_pred(:,4)));
    end
    
%     fprintf('i=%d,SSE [DEM,EM,SS,SS+PEM]:%f,%f,%f,%f\n',expt,...
%         SSE_theta(1,expt),SSE_theta(2,expt),SSE_theta(3,expt),SSE_theta(4,expt));
%     disp(sum(SSE_theta,2)')
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     expt
%     theta_convg_1(expt,:) = brain.memory.theta(3,:);
%     theta_convg_2(expt,:) = brain.memory.theta(4,:);
%     theta_convg_3(expt,:) = brain.memory.theta(6,:);

    
%     figure(54); 
%     hold on; plot3(brain.prior.theta(1),brain.prior.theta(2),brain.FreeE(1),...
%         '*','color','#77AC30','Linewidth',2);
%     hold on; plot3(brain.memory.theta(1,end-1),brain.memory.theta(2,end-1),...
%         brain.FreeE(end),'*','color','#7E2F8E','Linewidth',2);
%     hold on; plot3(brain.memory.theta(1,1:end-1),...
%         brain.memory.theta(2,1:end-1),brain.FreeE,'Linewidth',2); 
%     
%     figure(55);
% %     hold on; plot3(brain.A*ones(1,brain.nt),model.t,output.DEM_x(:,brain.nx*(brain.p+1)+1)');
%     hold on; xx = brain.A*ones(1,brain.nt);
%     yy = brain.B*ones(1,brain.nt);    zz = model.t;
%     col = output.DEM_x(:,brain.nx*(brain.p+1)+1)';  % This is the color
%     surface([xx;xx],[yy;yy],[zz;zz],[col;col],'facecol','no','edgecol',...
%         'interp','linew',2); colorbar; view(30,30)
    
    % Parameter SSE Plot for Exploration-Exploitation tradeoff
%     errP_Pp = [errP_Pp; sum((model.theta-brain.theta).^2) brain.prior.Pp(3,3)]; 
%     figure(117); hold on; plot(brain.prior.Pp(3,3),...
%                     sum((model.theta-brain.theta).^2),'r*'); 
%     set(gca,'xscale','log','yscale','log')

    % Hyperparameter estimation plot
%     errP_Pp = [errP_Pp; model.sigma_z^-2 exp(brain.h(1)) ...
%                         sum((model.theta-brain.theta).^2)   ]
%                     errP_Pp(:,3)
end

SSE(:,s_i) = sum(SSE_theta,2); 

% remove outlier data (mainly for EM) and sum all SSE
SSE_thresh = 100;
SSE_o(:,s_i) = sum(SSE_theta(:,~any(SSE_theta>SSE_thresh,1)),2); 
fprintf('Removed %d outlier SSE \n',sum(any(SSE_theta>SSE_thresh,1)));
disp(SSE_o(:,s_i)');
end

fprintf('FINISHED!! \n')

%%

%% Plot all free energy components, accuracy and complexity
figure(6);
titles = {'state PE','state entropy','noise entropy','param PE',...
          'param entropy','hyperP PE','hyperP entropy','Total FE'};
for i = 1:7
    subplot(2,4,i);
    plot(brain.temp(i,:) - brain.temp(i,1)); title(titles{i});
end
subplot(2,4,8); plot(brain.temp(8,:)-brain.temp(8,1)); title(titles{8}); hold on;

% figure(333); clf;
% plot(brain.accuracy - brain.accuracy(1,1)); hold on;
% plot(brain.accuracy - brain.accuracy(1,1) - ...
%     (brain.temp(8,:)-brain.temp(8,1))  );    hold on;
% plot(brain.temp(8,:)-brain.temp(8,1)); hold on;
% legend('Accuracy','Complexity','Free energy','location','NorthWest'); 
% xlabel('iterations');

%% 
function [SSE_theta,para_bench] = SSE_parameter(brain,model,output)

% Plot canonical observable form of parameters
sys_b = ss(brain.A,brain.B,brain.C,[]);
[csys_b, TTb] = canon(sys_b,'companion');
csys_b = [reshape(csys_b.A',1,[]) reshape(csys_b.B',1,[]) reshape(csys_b.C',1,[])];
[csys_m, TTm] = canon(ss(model.A,model.B,model.C,[]),'companion');
csys_m = [reshape(csys_m.A',1,[]) reshape(csys_m.B',1,[]) reshape(csys_m.C',1,[])];

% Subspace method for system identification
sys_s = n4sid(iddata(model.process_y,model.real_cause',model.sam_time),...
             brain.nx,n4sidOptions('N4weight','CVA'),...
             'Ts',0,'DisturbanceModel','estimate','Form','companion');
para_bench.s = sys_s;
csys_s = [reshape(sys_s.A',1,[]) reshape(sys_s.B',1,[]) reshape(sys_s.C',1,[])];

% Expectation Maximization Algorithm
[xxx_em,sys_em,~,~] = EM_for_ss(model.process_y',model.real_cause,...
                            model.sam_time,brain.nx);
csys_em = canon(d2c(sys_em),'companion');
para_bench.em = csys_em;
csys_em = [reshape(csys_em.A',1,[]) reshape(csys_em.B',1,[]) reshape(csys_em.C',1,[])];

% PEM after subspace method
sys_p = pem(iddata(model.process_y,model.real_cause',model.sam_time),sys_s);
para_bench.p = sys_p;
csys_p = [reshape(sys_p.A',1,[]) reshape(sys_p.B',1,[]) reshape(sys_p.C',1,[])];

% If the model order was increased internally, set everything to zero to
% avoid unfair comparison of estimators during SSE comparison
if size(csys_s,2)~=size(csys_m,2) || size(csys_p,2)~=size(csys_m,2) || ...
        size(csys_em,2)~=size(csys_m,2)
    csys_m = zeros(size(csys_m)); 
    csys_b = csys_m; csys_em = csys_m; csys_s = csys_m; csys_p = csys_m;
    fprintf('The model order increased, negative poles error \n');
end

SSE_theta = [sum((csys_m-csys_b).^2); sum((csys_m-csys_em).^2);...
             sum((csys_m-csys_s).^2); sum((csys_m-csys_p).^2)];
% fprintf('SSE [DEM,EM,SS,SS+PEM]:%f,%f,%f,%f\n',SSE_theta(1),SSE_theta(2)...
%                                               ,SSE_theta(3),SSE_theta(4));

fig11 = figure(7); clf; subplot(3,2,1:2);
bar([csys_m' csys_b' csys_em' csys_s' csys_p']); 
legend('Ideal','DEM','EM','SS','SS+PEM','location',...
    'best','orientation','vertical','fontsize',6);
title('Estimate of canonical observable parameters')
% writeasGIF(fig11,para)

% Compare the state esimates in the canonical space
subplot(3,2,3:4); hold on;
h1 = plot(model.t,(TTm*model.ideal_x')','b'); hold on;
h2 = plot(model.t,(TTb*output.DEM_x(:,1:brain.nx)')','r'); hold on;
legend([h1(1), h2(1)],'model can state','brain can state');
title('State estimation in canonical space');

% subplot(3,2,5); hold on;
% kkk = (brain.Da*output.DEM_x(:,1:brain.nx*(brain.p+1))')';
% h1 = plot(model.t,model.noise_w','r'); hold on;
% h2 = plot(model.t,kkk(:,1:brain.nx) - (brain.A*output.DEM_x(:,1:brain.nx)'+brain.B*...
%     output.DEM_x(:,brain.nx*(brain.p+1)+[1:brain.nu])')','b:'); hold on;
% legend([h1(1), h2(1)],'w','w est','location','best');
% title('Process noise estimation');
% 
% subplot(3,2,6); hold on;
% h1 = plot(model.t,model.noise_z','r');
% h2 = plot(model.t,model.process_y - (brain.C*output.DEM_x(:,1:brain.nx)')','b:'); hold on;
% legend([h1(1), h2(1)],'z','z est','location','best');
% title('Observation noise estimation');



% Calculate the VAF - Variance accounted for
% yyy_b = (brain.C*output.DEM_x(:,1:brain.nx)')';
% yyy_em = (sys_em.C*xxx_em)';                           % Output predicted by EM

% Validation data for output prediction under same input
% sys_r = ss(model.A,model.B,model.C,[]);
% yyy_r = lsim(sys_r,model.real_cause,model.t);
% yyy_b = lsim(sys_b,model.real_cause,model.t);
% yyy_em= lsim(sys_em,model.real_cause,model.t);
% yyy_s = lsim(sys_s,model.real_cause,model.t);
% yyy_p = lsim(sys_p,model.real_cause,model.t);
% 
% figure(8); clf;
% h1 = plot(yyy_r,'r'); hold on;
% h2 = plot(yyy_b,'b'); hold on;
% h3 = plot(yyy_em,'g'); hold on; 
% h4 = plot(yyy_s,'m'); hold on; 
% h5 = plot(yyy_p,'k'); hold on; 
% 
% % VAF w.r.t training data
% fprintf('VAF DEM: %f\n',mean(1 - (var(model.process_y - yyy_b) ./ var(model.process_y))))
% fprintf('VAF  EM: %f\n',mean(1 - (var(model.process_y - yyy_em) ./ var(model.process_y))))
% fprintf('VAF  SS: %f\n',mean(1 - (var(model.process_y - yyy_s) ./ var(model.process_y))))
% fprintf('VAF SS+PEM: %f\n',mean(1 - (var(model.process_y - yyy_p) ./ var(model.process_y))))
% legend([h1(1) h2(1) h3(1) h4(1) h5(1)],'Real output','DEM y\_pred',...
%     'EM y\_pred','SS y\_pred','SS+PEM y\_pred');
% 
% fprintf('SSE_y [DEM,EM,SS,SS+PEM]: %f,%f,%f,%f\n',sum((yyy_r-yyy_b).^2,'all'),...
%    sum((yyy_r-yyy_em).^2,'all'),sum((yyy_r-yyy_s).^2,'all'),...
%    sum((yyy_r-yyy_p).^2,'all'));

end



%%
% figure; hBar = bar(SSE_mean_x); hold on; 
% errorbar(bsxfun(@plus, hBar(1).XData, [hBar.XOffset]')',SSE_mean_x,SSE_std_x,'k.')
% xticklabels({'\sigma=0.1','\sigma=0.3','\sigma=0.5','\sigma=0.7','\sigma=0.9'})
% ylabel('SSE of state estimation');
% % legend({'p = 0','p = 1','p = 2','p = 3','p = 4','p = 5','p = 6'},...
% %     'Orientation','horizontal','NumColumns',4);
% % legend('KF: unknown input','UIO: unknown input','DEM: unknown input',...
% %     'KF: known input','DEM: known input');
% legend('KF','DEM(\sigma=0)','DEM(\sigma=\sigma)');
% set(findall(gcf,'-property','FontSize'),'FontSize',12)

%%
% print_results(SSE,if_UIO,if_cause);


% plot_results(output,model,brain,if_UIO,if_cause);


% figure;  plot(1*ones(1,iter),SSE_x_kalman,'b*');
% hold on; plot(2*ones(1,iter),SSE_x_UIO,'c*');
% hold on; plot(3*ones(1,iter),SSE_x_DEM,'g*'); 
% hold on; plot(4*ones(1,iter),SSE_x_kalmv,'r*','MarkerSize',5); 
% hold on; plot(5*ones(1,iter),SSE_x_DEMv,'m*');
% 
% xlim([0,6]); set(gca,'XTick',[]); legend('KF','UIO','DEM','KF\_v','DEM\_v');

% figure; plot(1*ones(1,iter),SSE_v_UIO,'r*');
% hold on; plot(2*ones(1,iter),SSE_v_DEM,'b*'); 
% xlim([0,3]); set(gca,'XTick',[]); legend('UIO\_cause','DEM\_cause');

%%
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_Y_friston.mat');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_pX_friston.mat');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_Xest_friston.mat');
% figure; plot(process_x,'-'); hold on; plot(DEM_pX_friston','-.'); 
% figure; plot(process_y,'-'); hold on; plot(DEM_Y_friston','-.')
% figure; plot(DEM_x(:,1:2),'-'); hold on; plot(DEM_Xest_friston','-.')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman filter with causes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [kalmf, Lk, Pk] = kalmd(ss(A,[B,eye(nx)],C,zeros(ny,nx+1)),...
%                inv(P_brain_w),... +A*B*exp(0)*B'*A',...
%                inv(P_brain_z),sam_time);
% kalmfv_x = zeros(size(t,2),size(kalmf.C,2));
% for i = 2:size(t,2)
%     kalmfv_x(i,:)=(kalmf.A*kalmfv_x(i-1,:)' + kalmf.B*...
%                                     [real_cause(:,i); process_y(i,:)']);
% end
% 

%% All function definitions

function brain = update_parameters(DEM_x,brain,if_MF_p)
    
    np = (brain.nx^2+brain.nx*brain.nu+brain.nx*brain.ny); % no:of parameters
    M = zeros(brain.nx*(brain.p+1),np);
    N = zeros(brain.ny*(brain.p+1),np);
    AA = zeros(np); BB = zeros(np,1);
    EE = zeros((brain.nx+brain.ny)*(brain.p+1) + brain.nu*(brain.d+1));
%     ECEu = EE;  ECEp = EE; 
    EEE1 = zeros(brain.ny*(brain.p+1)); EEE2 = zeros(brain.nx*(brain.p+1));
    
    iquC = 0;                                 % conditional information
%     brain.Pp = 0*brain.Pp;                    % set para precision to 0
    
    theta = [reshape(brain.A',[],1); reshape(brain.B',[],1); ...
             reshape(brain.C',[],1)];
         
    % Precision matrices
    brain.Pn = blkdiag(brain.V0,brain.W0);    % Precision matrix of noises
    brain.Pu = [brain.Ct'*brain.V0y*brain.Ct + brain.D_A'*brain.W0*brain.D_A ...
            -brain.D_A'*brain.W0*brain.Bt;...
            -brain.Bt'*brain.W0*brain.D_A   brain.V0v+brain.Bt'*brain.W0*brain.Bt];
         
    for j = 1:brain.nt
        for i = 1:brain.p+1
            temp = DEM_x(j, 1+(i-1)*brain.nx:i*brain.nx);
            M(1+brain.nx*(i-1):brain.nx*i, 1:brain.nx^2) = ...
                            kron(eye(brain.nx),temp);    
            N(1+brain.ny*(i-1):brain.ny*i, brain.nx^2+brain.nx*brain.nu+1:np) = ...
                            kron(eye(brain.ny),temp);
        end
        for i = 1:brain.d+1
            temp = DEM_x(j, brain.nx*(brain.p+1)+(i-1)*brain.nu+[1:brain.nu]);
            M(1+brain.nx*(i-1):brain.nx*i, brain.nx^2+1: ...
                brain.nx^2+brain.nx*brain.nu) = kron(eye(brain.nx),temp); 
        end
        AA = AA + N'*brain.V0y*N + M'*brain.W0*M;
        BB = BB + [N'*brain.V0y M'*brain.W0*brain.Da]*[brain.Y_embed(1:brain.ny*...
            (brain.p+1),j) ;DEM_x(j,1:brain.nx*(brain.p+1))'];
        
        % Precisions of parameters
        brain.Pp = AA;
                
        % Accumulate terms for hyperparameter estimation
        E = [brain.Y_embed(:,j) + [-brain.Ct*DEM_x(j,1:brain.nx*(brain.p+1))'; ...
              DEM_x(j,brain.nx*(brain.p+1)+1:end)']; brain.D_A*...
              DEM_x(j,1:brain.nx*(brain.p+1))'...
              - brain.Bt*DEM_x(j,brain.nx*(brain.p+1)+1:end)'];
        EE = EE + E*E';
%         Eu = [blkdiag(-brain.Ct, eye(brain.nu*(brain.d+1))); brain.D_A -brain.Bt];
%         Ep = [-N; zeros(brain.nu*(brain.d+1),np); -M];
%         ECEu = ECEu + Eu*spm_inv(brain.Pu)*Eu';
%         ECEp = ECEp + Ep*spm_inv(brain.Pp)*Ep';
                    
        Px = spm_inv(brain.Pu);
        EEE1 = EEE1 + (brain.Y_embed(1:brain.ny*(brain.p+1),j) - brain.Ct*DEM_x(j,1:brain.nx*(brain.p+1))')...
            *(brain.Y_embed(1:brain.ny*(brain.p+1),j) - brain.Ct*DEM_x(j,1:brain.nx*(brain.p+1))')'...
            + brain.Ct*Px(1:brain.nx*(brain.p+1),1:brain.nx*(brain.p+1))*brain.Ct' ...
            + N*spm_inv(brain.Pp)*N';
        EEE2 = EEE2 + (brain.D_A*DEM_x(j,1:brain.nx*(brain.p+1))'...
              - brain.Bt*DEM_x(j,brain.nx*(brain.p+1)+1:end)')*...
              (brain.D_A*DEM_x(j,1:brain.nx*(brain.p+1))'...
              - brain.Bt*DEM_x(j,brain.nx*(brain.p+1)+1:end)')'...
              + [brain.D_A -brain.Bt]*spm_inv(brain.Pu)*[brain.D_A -brain.Bt]' ...
              + M*spm_inv(brain.Pp)*M';
        
        % Update parameter precisions
        
    end
%     figure(1); hold on; plot(DEM_x(:,1))
 
    % conditional covariance of states (Entropy - log det)
    iquC = brain.nt*spm_logdet(spm_inv(brain.Pu));%log(det(inv(Eu'*brain.Pn*Eu)));
%     disp(iquC)
      
    AA = (-AA - brain.prior.Pp);  % Be careful when you add mean field terms here
    BB = BB + brain.prior.Pp*brain.prior.theta;
    
    % Mean field terms for parameter estimation
    [dWdp,dWdpp] = mean_field_param(brain.D_A,brain.Bt,brain.Ct,brain.dE,...
        brain.p,brain.d,brain.nx,brain.ny,brain.nu,brain.Pu,brain.Pn);
    
    learn_rate_p = 10;
    dFdp = learn_rate_p*((AA*theta + BB) + if_MF_p*dWdp*brain.nt); 
    dFdpp = learn_rate_p*(AA + if_MF_p*dWdpp*brain.nt); 
      
    % Loop over for hyperparameter estimation
    for kk =1:8
        
        brain.V0y =  brain.Q_V0y*exp(brain.h(1)); %brain.Q_V0y +  brain.Q_V0y*exp(brain.h(1));
        brain.V0  = blkdiag(brain.V0y,brain.V0v);
        brain.W0  =  brain.Q_W0 *exp(brain.h(2)) ; %brain.Q_W0  +  brain.Q_W0 *exp(brain.h(2));
        brain.Pn  = blkdiag(brain.V0,brain.W0);
        
%         Q1 = blkdiag(brain.Q_V0y*exp(brain.h(1)),brain.Q_V0v*0);
%         Q2 = brain.Q_W0*exp(brain.h(2));
%         Q =  blkdiag(Q1,Q2);
                
        % + ECEu + ECEp
%         UWW = EE + ECEu + ECEp - brain.nt*pinv(brain.Pn);       
%         dFdh = -.5*[trace(blkdiag(Q1,Q2*0)*UWW); trace(blkdiag(Q1*0,Q2)*UWW)]...
%             - brain.prior.Ph*(brain.h - brain.prior.h)
        
        learn_rate_h = 1;
        dFdh = learn_rate_h*(-.5*[trace(brain.V0y*EEE1); trace(brain.W0*EEE2)] ...
            - brain.prior.Ph*(brain.h - brain.prior.h) ...
            + 0.5*brain.nt*[size(brain.V0y,1); size(brain.W0,1)]);

%         Pinv_zv = pinv(blkdiag(brain.V0y,brain.V0v));
%         dFdhh = -.5*blkdiag(trace(Q1*Pinv_zv*Q1*Pinv_zv*brain.nt),...
%             trace(Q2*pinv(brain.W0)*Q2*pinv(brain.W0)*brain.nt)) - brain.prior.Ph;
        dFdhh = learn_rate_h*(-.5*diag([size(brain.V0y,1),size(brain.W0,1)])*...
                                    brain.nt - brain.prior.Ph);        
        dh = (expm(dFdhh*4) - eye(size(dFdhh)))*pinv(dFdhh)*dFdh;
                dh    = max(min(dh,2),-2);
        brain.h = brain.h + dh;
               
        if (dFdh'*dh < .02) || (norm(dh,1) < .02), break, end
    end
    
    brain.h;
    
    brain.Pp = -AA;
    brain.Ph = -dFdhh;
       
    brain.FreeE(end+1) = .5*(- trace(brain.Pn*EE) ...           % states (u)
        - (brain.p+1)*brain.ny*log(2*pi)*brain.nt ...           % constant
        + log(det(brain.Pn))*brain.nt ...                       % entropy - error
        + iquC/(1) ...                                          % entropy q(u)
        - trace(theta'*brain.prior.Pp*theta)  ...               % parameters (p)
        - trace((brain.h-brain.prior.h)'*brain.prior.Ph*...
        (brain.h-brain.prior.h))  ...                           % hyperparameters (h)
        + spm_logdet(spm_inv(brain.Pp)*brain.prior.Pp)   ...         % entropy q(p)
        + spm_logdet(spm_inv(brain.Ph)*brain.prior.Ph)  );           % entropy q(h)
    
    brain.FreeE(end) - brain.FreeE(1);
    brain.temp(:,end+1)= [.5*(  - trace(brain.Pn*EE) );...
                          .5*(  + iquC/(1) );...
                          .5*(  + log(det(brain.Pn))*brain.nt  );...
                          .5*(  - trace(theta'*brain.prior.Pp*theta)  );...
                          .5*(  spm_logdet(spm_inv(brain.Pp)*brain.prior.Pp) );...
                          .5*(   - trace((brain.h-brain.prior.h)'*brain.prior.Ph*...
                                        (brain.h-brain.prior.h))   );...
                          .5*(  spm_logdet(spm_inv(brain.Ph)*brain.prior.Ph)  );...
                          brain.FreeE(end)   ];
    brain.accuracy(:,end+1) = .5*( -trace(brain.V0y*EE(1:brain.ny*(brain.p+1),...
                            1:brain.ny*(brain.p+1))) + log(det(brain.V0y))*brain.nt );
    
    %     sys = c2d(ss(AA,BB,eye(np),[]),1);
%     theta = sys.A*theta + sys.B;

    % Update parameters if there is an increase in free energy  
    if (size(brain.FreeE,2) == 1 || brain.FreeE(end) > brain.FreeE(end-1))
        d_theta = (expm(dFdpp) - eye(size(AA)))*pinv(dFdpp)*dFdp;
        theta = theta + d_theta;
        A_new = reshape(theta(1:brain.nx^2),brain.nx,[])';
        B_new = reshape(theta(brain.nx^2+1:brain.nx^2+brain.nx*brain.nu),[],brain.nx)';
        C_new = reshape(theta(brain.nx^2+brain.nx*brain.nu+1:end),brain.nx,[])';
        
        brain.A = A_new; brain.B = B_new; brain.C = C_new; C_new;
        brain.theta = theta;
        brain.S_theta =  sqrt(1./diag(brain.Pp));
    else
%         disp("Free energy decreased...");
    end  
        
    % save parameter update for plotting
    brain.memory.theta(:,end+1) = brain.theta;
    brain.memory.S_theta(:,end+1) = brain.S_theta;
    brain.memory.h(:,end+1) = brain.h;  % save h updates for plotting

%     figure(777); 
%     plot(size(brain.memory.theta,2),brain.S_theta,'r*');
%     plot(-2:.1:2,normpdf(-2:.1:2,brain.theta(1),brain.S_theta(1)));
%     hold on; 
end

function [dWdp,dWdpp] = mean_field_param(D_A,Bt,Ct,dE,p,d,nx,ny,nu,Pu,Pn)
%% Calculate mean field terms required for parameter estimation

    ntheta = nx*nx + nx*nu + nx*ny;
    Eu = [blkdiag(-Ct, eye(nu*(d+1))); D_A -Bt];
    dWdp = zeros(ntheta,1);
    for i = 1:ntheta
        dWdp(i,1) = -.5*trace(pinv(Pu)* dE.dXp{i}'* Pn*Eu);
    end
    
    dWdpp = zeros(ntheta,ntheta);
    for i = 1:ntheta
        for j = 1:ntheta
            dWdpp(i,j) = -.5*trace(pinv(Pu)* dE.dXp{i}'* Pn* dE.dXp{j});
        end
    end
    
end

function [dWdX,dWdXX] = mean_field_state(X,dE,p,d,nx,ny,nu,Pp,V0y,W0)
    
    X = [X; zeros(nu*(p-d),1)];         % resize to account for p-d zeros
    dNdX = dE.dNdX; dMdX = dE.dMdX;
    
    % Prepare N and M matrices
    N = []; M = [];
    for i = 1:p+1
       N = [N; zeros(ny,nx*nx + nx*nu) kron(eye(ny,ny),X((i-1)*nx+1:i*nx,1)')];
       M = [M; kron(eye(nx,nx),X((i-1)*nx+1:i*nx,1)') ...
               kron(eye(nx,nx),X(nx*(p+1)+ ((i-1)*nu+1:i*nu),1)') zeros(nx,nx*ny)];   
    end
           
    nX = nx*(p+1)+nu*(d+1);
    dWdX = zeros(nX,1);
    for i = 1:nX
        dWdX(i,1) = -.5*trace(pinv(Pp)* (dNdX{i}'*V0y*N + dMdX{i}'*W0*M) );  
    end
    
    dWdXX = zeros(nX,nX);
    for i = 1:nX
        for j = 1:nX
            dWdXX(i,j) = -.5*trace(pinv(Pp)* (dNdX{i}'*V0y*dNdX{j} + dMdX{i}'*W0*dMdX{j}) );
        end
    end
end

function dE = Error_pX(p,d,nx,ny,nu)
%% Prepare derivatives of M and N
    
    ntheta = nx*nx + nx*nu + nx*ny;
    
    % state terms
    temp = {};
    for i = 1:nx
        temp{i} = zeros(1,nx); temp{i}(i) = 1;
    end   
    for i = 1:p+1
        for j = 1:nx
            dNdX{(i-1)*nx+j} = sparse(ny*(p+1),ntheta);
            dNdX{(i-1)*nx+j}((i-1)*ny+1:i*ny, nx*nx+nx*nu+1: ntheta) = ...
                                                    kron(eye(ny),temp{j});         
            dMdX{(i-1)*nx+j} = sparse(nx*(p+1),ntheta);
            dMdX{(i-1)*nx+j}((i-1)*nx+1:i*nx, 1:nx*nx) = kron(eye(nx),temp{j});
        end
    end
    
    % input terms
    temp = {};
    for i = 1:nu
        temp{i} = zeros(1,nu); temp{i}(i) = 1;
    end
    for i = 1:d+1
        for j = 1:nu
            dNdX{nx*(p+1)+(i-1)*nu+j} = sparse(ny*(p+1),ntheta);
            dMdX{nx*(p+1)+(i-1)*nu+j} = sparse(nx*(p+1),ntheta);
            dMdX{nx*(p+1)+(i-1)*nu+j}((i-1)*nx+1:i*nx, nx*nx+1:nx*nx+nx*nu) = ...
                                                    kron(eye(nx),temp{j});
        end
    end
    
    % Swap cells and columns to get dEdXp, used by parameter mean field
    dE.dXp = cell(1,ntheta);
    for i = 1:ntheta
        for j = 1:size(dNdX,2)
            dE.dXp{i}(:,j) = [-dNdX{j}(:,i); zeros(nu*(d+1),1); -dMdX{j}(:,i)];
        end
    end
    
    dE.dNdX = dNdX; dE.dMdX = dMdX; 
end

function brain = init_brain(brain)

brain.FreeE = [];
brain.memory.h = brain.prior.h;
brain.memory.theta = brain.prior.theta;
brain.memory.S_theta = sqrt(1./diag(brain.prior.Pp));
brain.temp  = []; 
brain.accuracy = [];

brain.nx = size(brain.A,1);
brain.ny = size(brain.C,1);
brain.nu = size(brain.B,2);

brain = set_brain(brain);

brain.Pz = eye(brain.ny)/brain.sigma_z^2;
brain.Pw = eye(brain.nx)/brain.sigma_w^2;

% sigma_brain_w = (diag([1.2341e-04,1.2341e-04,1.2341e-04]));
% sigma_brain_z = 7.5046e-06;
% P_brain_w = diag(1./(diag(sigma_brain_w).^2));
% P_brain_z = diag(1./(diag(sigma_brain_z).^2));

[brain.W0,brain.V0y,brain.V0v] = precision_matrix(brain.s,brain.Pw,...
                 brain.Pz,brain.sigma_v,brain.p,brain.d,brain.nu);
brain.V0 = blkdiag(brain.V0y,brain.V0v);

brain.prior.V0y = brain.V0y;
brain.prior.W0 = brain.W0;

% Prepare generalized precision matrices with Identiry matrices
[brain.Q_W0, brain.Q_V0y,brain.Q_V0v] = precision_matrix(brain.s,...
    eye(size(brain.Pw)),eye(size(brain.Pz)),brain.sigma_v,brain.p,...
    brain.d,brain.nu);
brain.Q_V0v = brain.Q_V0v*0;    % No hyperparameter updation for v

% Evaluate dEdpX for mean field terms
brain.dE = Error_pX(brain.p,brain.d,brain.nx,brain.ny,brain.nu);      

end

function brain = set_brain(brain)
%% Reset the brain with new parameter values

brain.At = kron(eye(brain.p+1),brain.A);
brain.Bt = kron(eye(brain.p+1,brain.d+1),brain.B);
brain.Ct = kron(eye(brain.p+1),brain.C);

T = toeplitz(zeros(1,brain.p+1),[0 1 zeros(1,brain.p-1)]);
if brain.p==0; T=0;end
brain.Da = kron(T,eye(brain.nx));
T = toeplitz(zeros(1,brain.d+1),[0 1 zeros(1,brain.d-1)]);
if brain.d==0; T=0;end
brain.Dv = kron(T,eye(brain.nu));
brain.D_A = brain.Da-brain.At;

end

function [model] = generative_process(model,brain,if_dataset)

model.nx = size(model.A,1); model.ny = size(model.C,1); model.nu = size(model.B,2);

Tt  = 0:model.sam_time:model.t_end-model.sam_time;
model.Pz = eye(model.ny)/model.sigma_z^2;
model.Pw = eye(model.nx)/model.sigma_w^2;

if if_dataset==0   
    % generate noise
    [noise_w, noise_z] = make_noise(model.s,model.Pw,model.Pz,Tt,...
        model.sam_time,model.nx,model.ny,brain.nt);
    model.noise_w = noise_w;
    model.noise_z = noise_z;
    
    % s_est = estimate_smoothness(noise_z,Tt);
    
    % end
    % plot(s_realA,s_est,'x'); hold on; plot([0 1],[0 1])
    
    
    % generative process
    process = ss(model.A,[model.B   eye(model.nx)  zeros(model.nx,model.ny)],...
        model.C,[zeros(model.ny,model.nu) zeros(model.ny,model.nx) eye(model.ny)]);
    process = c2d(process,model.sam_time,'zoh');
    % [process_y,~,process_x] =lsim(process,[real_cause; noise_w; noise_z]...
    %                                ,t, zeros(2,1),'zoh');
    
    [process_x, process_y] = generate_data(model.A,model.B,model.C,noise_w,noise_z,...
        model.real_cause,model.t,model.sam_time,model.p,model.d);
    model.ideal_x = process_x;
    model.process_x = process_x;
    model.process_y = process_y;
end

end

function output = observer(model,brain,if_UIO,UIO_gamma,if_cause,...
    if_dataset,if_predict_y,if_MF_X)

% Unknown Input Observer
if if_UIO
    [output.UIO_x_est, output.UIO_v_est] = UIO_estimator(...
        c2d(ss(brain.A,brain.B,brain.C,0),model.sam_time,'zoh'),...
        model.ideal_x',model.process_y',brain.nt,model.real_cause,...
        if_dataset,UIO_gamma);
end

% DEM with unknown causes
[output.DEM_t,output.DEM_x] = D_step(brain.A,brain.B,brain.C,brain.dE,...
    brain.Y_embed,brain.V0,brain.W0,brain.Pp,...
    brain.At,brain.Bt,brain.Da,brain.Dv,brain.nx,brain.ny,brain.nu,brain.nt,...
    brain.p,brain.d,model.t,model.sam_time,if_predict_y,if_MF_X);

% Kalman Filter with unknown causes                  
if_causes = 0;                   
output.kalman_x = kalman_discrete(brain.A,brain.B,brain.C,model.process_y,...
    brain.Pz,brain.Pw,model.sam_time,model.t,model.real_cause,if_causes);

if if_cause
    % D step with known causes
    output.DEMv_x = D_step_causes(brain.A,brain.D_A,brain.B,brain.Da,brain.Bt,brain.Ct,...
        brain.V0y,brain.W0,brain.Y_embed,model.real_cause,...
        model.t,model.sam_time,brain.nt,brain.ny,brain.nu,brain.p,brain.d);
    
    % Kalman Filter with causes
    output.kalmfv_x = KF_causes(brain.A,brain.B,brain.C,brain.ny,model.sam_time,...
        brain.Pw,brain.Pz,model.real_cause,model.process_y,model.t,brain.nx,brain.nu);
end

end

function f = find_s(s_real,noise_z,Tt)
K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;

N = round(.25*size(noise_z,2));
corrupt_sig = noise_z*pinv(K);

corrupt_sig = corrupt_sig((end-N)/2+1:(end+N)/2);
f = sum((autocorr(corrupt_sig,'NumLags',30).^2)); % ,'NumLags',round(N/2)-1
% f = sum((autocorr(noise_z*pinv(K),'NumLags',50)>.2));

end

function y_t = n_step_ahead_prediction(n,x0,A,B,C,sam_time,u,nt)

    sys = c2d(ss(A,B,C,[]),sam_time); % ss(A,B,C,[],sam_time);
    x_t = x0;  
    y_t = zeros(size(C,1),n);
    for ii = 1:n
        x_t1 = sys.A*x_t + sys.B*u(:,nt+ii-1);
        y_t(:,ii) = sys.C*x_t;
        x_t = x_t1;
    end

end

function [z_f, d_f] = UIO_estimator(sys,x_real,y_real,k,d,if_dataset,UIO_gamma)

A = sys.A; C = sys.C; D = sys.D;
B1 = zeros(size(A,1),1); B2 = sys.B; 

% A = [-.25 1; -.5 -.25];
% B = [1; 0];
% C = [.125 .1633; .125 .0676; .125 -.0676; .125 -.1633];

% A = [0.9323 0.0185; -0.0092 0.9138];
% B1 = [.1; 0];
% B2 = [.1; .1];
% C = [5 0];
% D = 0.1;
% H = [0 1];

% A = [0.9323 0.0194; -0.0097 1.01];
% B1 = [.1; 0];
% B2 = [.1; .1];
% C = [5 0];
% D = 0;

nx = size(A,1); ny = size(C,1); nu = size(B2,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method 1, as per the paper, solve the LMI directly and tune gamma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if if_dataset == 1
    P = sdpvar(nx,nx);
    Q = sdpvar(nu,nu);
    M = sdpvar(nu,nu);
    G = sdpvar(nx,ny);
    % gamma = 100;
    
    AA = [-P         zeros(nx,1)             (P*A-G*C)'     -(M*B2'*C'*C)'; ...
        zeros(1,nx)   -Q                      B2'*(P*A-G*C)' Q-B2'*(M*B2'*C'*C)'; ...
        (P*A-G*C)     (B2'*(P*A-G*C)')'       -P             zeros(nx,1); ...
        -(M*B2'*C'*C) Q'-(B2'*(M*B2'*C'*C)')' zeros(1,nx)    -Q];
    
    F = [P>=0, Q>=0, M>=0, AA<=0];
    optimize(F);
    P = value(P); Q = value(Q);
    M = value(M);
    G = value(G);
    L = inv(P)*G
    gamma = M*inv(Q)
    L = value(L)
    gamma = value(gamma)*1200000  % tune for best gamma; motor = gamma*1200000

else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Method 2, as per paper, stable observer design through eigen values
    %          tune parameter UIO_gamma for best performance
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = sdpvar(nx,ny);
    gamma = sdpvar(nu,nu);
    AAA = [A-L*C (A-L*C)*B2; -gamma*B2'*C'*C eye(nu,nu)...
        - gamma*B2'*C'*C*B2];
    % optimize([sum(abs(eig(A-L*C)))<=1.75, ...
    %     sum(abs(eig(eye(ny)-C*B2*gamma*B2'*C')))<=.3, gamma>=.01, L>=0])
    optimize([sum(abs(eig(AAA)))<=size(AAA,1), gamma>=UIO_gamma])
    % optimize([max(abs(eig(AAA)))<=1, gamma>=UIO_gamma])
    
    L = value(L)
    gamma = value(gamma)   
    % value(abs(eig(AAA)))
end
% setlmis([])
% P = lmivar(1,[nx 1]);  Q = lmivar(1,[nu 1]);
% M = lmivar(2,[nu nu]); G = lmivar(2,[nx ny]);
% 
% lmiterm([1 1 1 P],-1,1); lmiterm([1 1 3 -P],A',1); lmiterm([1 1 3 -G],C',-1);
% lmiterm([1 1 4 -M],-C'*C*B2,1);
% lmiterm([1 2 2 Q],-1,1);
% lmiterm([1 2 3 -P],B2'*A',1); lmiterm([1 2 3 -G],B2'*C',-1);
% lmiterm([1 2 4 Q],1,1); lmiterm([1 2 4 -M],-B2'*C'*C*B2,1);
% lmiterm([1 3 3 P],-1,1);
% lmiterm([1 4 4 Q],-1,1);
% 
% lmiterm([-2 1 1 P],1,1);
% lmiterm([-3 1 1 Q],1,1);
% lmiterm([-4 1 1 M],1,1);  
% 
% LMISYS = getlmis;
% options = [0,0,1000,0,0];
% [tmin, solution] = feasp(LMISYS);%,options,-1);
% P = dec2mat(LMISYS,solution,P);
% Q = dec2mat(LMISYS,solution,Q);
% M = dec2mat(LMISYS,solution,M);
% G = dec2mat(LMISYS,solution,G);
% L = inv(P)*G;
% gamma = M*inv(Q);

% L = [.1886; .0969]; gamma = .9583;

% friston's system
% L = .4*L
% gamma = 15*gamma

% spring damper system
% L = .45*L;
% gamma = 4050*gamma;

% AAA = [A-L*C (A-L*C)*B2; -gamma*B2'*C'*C eye(size(gamma,1),size(B2,2))...
%                                     - gamma*B2'*C'*C*B2];
% abs(eig(AAA))
% fprintf('Stable observer? %d\n',sum(abs(eig(AAA))<1) == size(AAA,1));

% k = 500;
% d = zeros(1,k); 
u = zeros(1,k);
% d(1,125:320) = exp(-.0025*((125:320)-250).^2);%sin((125:320)/20);
% u(1,1:k) = 0;%sin((1:k)/20);
% x_real = zeros(nx,k);
% y_real = zeros(ny,k);
z_real = x_real;
% for i = 1:k
% %    x_real(:,i+1) = A*x_real(:,i) + B1*u(:,i) + B2*d(:,i);
% %    y_real(:,i) = C*x_real(:,i) + D*u(:,i);
%    z_real(:,i) = H*x_real(:,i); 
% end

d_f = zeros(nu,k);
x_f = zeros(nx,k);
y_f = zeros(ny,k);
z_f = zeros(nx,k);

d_f0 = .01;
d_f(:,1) = d_f0 + gamma*B2'*C'*(y_real(:,1) - C*B2*d_f0); % x_f(1) = 0
x_f(:,2) = A*B2*d_f0 + B1*u(:,k) + L*(y_real(:,1) - C*B2*d_f0);

for i = 2:k-1
    y_f(:,i) = C*x_f(:,i) + D*u(:,i) + C*B2*d_f(:,i-1);
    d_f(:,i) = d_f(:,i-1) + gamma*B2'*C'*(y_real(:,i) - y_f(:,i));
    z_f(:,i) = x_f(:,i) + B2*d_f(:,i-1);
    x_f(:,i+1) = A*x_f(:,i) + B1*u(:,i) + A*B2*d_f(:,i-1) + ...
                                     L*(y_real(:,i) - y_f(:,i));
end
y_f(:,k) = C*x_f(:,k) + D*u(:,k) + C*B2*d_f(:,k-1);
d_f(:,k) = d_f(:,k-1) + gamma*B2'*C'*(y_real(:,k) - y_f(:,k));
z_f(:,k) = x_f(:,k) + B2*d_f(:,k-1);

d_f(:,1:k-1) = d_f(:,2:k);
z_f(:,1:k-1) = z_f(:,2:k);

% figure; plot(d'); hold on; plot(d_f')
% sum((d-d_f).^2)
end

function [t,real_cause,process_x,process_y,ideal_y,current,voltage,Ts,A,B,C] ...
    = DC_motor_data

load C:\Users\ajithanilmeera\Desktop\DEM\DC_motor_data\R1\gbn_data_new_maxon_nogear;
% load(strcat('C:\Users\LocalAdmin\Desktop\DC_motor_data\',...
%  'motor_data\motor without gearbox\random_withoutgear_2schijf_100'));

% s = tf('s');
% H = G/s;
real_cause = actuatorgain*U';  % -actuatorgain*V'
t = t';
% ideal_y = lsim(H,real_cause,t);
process_y = Y*sensorgain;
% plot(t,process_y,t,ideal_y)
% state_sp = ss(H);

A = [0 1 0; 0 -b/J K/J; 0 -K/L -R/L];
B = [0 ;0 ;1/L];
C = [1 0 0];
[ideal_y,~,process_x] = lsim(ss(A,B,C,0),real_cause,t);
current = -I*actuatorgain;
voltage = -actuatorgain*V';

% state_sp_ = c2d(state_sp,Ts);
% process_x = zeros(size(t,2),size(state_sp_.C,2));
% for i = 2:size(t,2)
%     process_x(i,:)=(state_sp_.A*process_x(i-1,:)' + state_sp_.B*real_cause(:,i))';
% end




% load data_generation\MeasData.mat;
% 
% z = z3;
% G = tfest(z,2,0);
% t = z.SamplingInstants';
% Ts = z.Ts;
% u = z.InputData;
% process_y = z.OutputData;
% 
% state_sp = ss(G);
% A = state_sp.A;
% B = state_sp.B;
% C = state_sp.C;
% 
% [~,fit,xxx0] = compare(z,G);
% [ideal_y,~,process_x] = lsim(c2d(state_sp,Ts),u,t,xxx0);
% real_cause = u';
% fprintf('Parameter fit = %f\n',fit);


end

function [Tt,model,brain] = figure_8_drone_data

        model.sam_time = 1;
        model.t_end = 201;%201 drone;
        Tt  = 0:model.sam_time:model.t_end-model.sam_time;
        model.t = Tt + model.sam_time;
        
        load('C:\Users\ajithanilmeera\Desktop\V2.5_wp_10_Flight01.mat');
        data_range = 20:220;   % 20:220 460:700 works with smooth i/p & o/p
        model.real_cause = ((RCOU(data_range,4:5))/190)'/6;  % 3:6
        model.real_cause = smoothdata(model.real_cause','gaussian',5)';
        
%         model.s = .5; %0.01 + exp(-16);
%         brain.s =  model.s;%0.01 + exp(-16);    % motor
%         
%         model.p = 2;
%         model.d = 1;
%         brain.p = 2; % motor data 5 % drone 2
%         brain.d = 1; % motor data 2 % drone 1
%         
%         model.sigma_w = exp(-8);%exp(-6);     %motor data
%         model.sigma_z = exp(-4);%exp(-13);  %motor data
%         brain.sigma_v = exp(-8); % exp(-8)
%         brain.sigma_w = exp(0);%model.sigma_w;
%         brain.sigma_z = exp(0);%model.sigma_z;%exp(-4);    
        
        model.process_y(:,1) = deg2rad(NKF1(data_range,3));           % Roll
        model.process_y(:,2) = deg2rad(NKF1(data_range,4));           % Pitch
        model.process_y(:,3) = unwrap(deg2rad(NKF1(data_range,5))); % Yaw
        model.process_y(:,4) = IMU(floor((data_range)*2.5),3);        % Ang vel X
        model.process_y(:,5) = IMU(floor((data_range)*2.5),4);        % Ang vel Y
        model.process_y(:,6) = IMU(floor((data_range)*2.5),5);        % Ang vel Z
        model.process_y(:,7) = IMU(floor((data_range)*2.5),6);       % Acc X
        model.process_y(:,8) = IMU(floor((data_range)*2.5),7);       % Acc Y
        model.process_y(:,9) = IMU(floor((data_range)*2.5),8);       % Acc Z
        model.process_y(:,10) = atan(deg2rad(POS(data_range,3)))*6371000/6; % X /6
        model.process_y(:,11) = atan(deg2rad(POS(data_range,4)))*6371000; % Y
        model.process_y(:,12) = (POS(data_range,6));                 % Z
        
        model.process_y = model.process_y - model.process_y(1,:);
        model.process_y = smoothdata(model.process_y,'gaussian',5);
               
        % Randomly selected model matrices
        while 1
            rng('shuffle')
            brain.ny = 12; brain.nx = 3;     % drone 9,2
            brain.nu = size(model.real_cause,1);
            model.A = -1+2*rand(brain.nx);
            model.B = -1+2*rand(brain.nx,brain.nu);
            model.C = -1+2*rand(brain.ny,brain.nx);
            if isstable(ss(model.A,model.B,model.C,0))
                break;
            end
        end
                        
        model.process_x = zeros(size(model.process_y,1),brain.nx);
end

function [Tt,u,x,y,ideal_y,sam_time,A,B,C,full_data] = ardrone_data

if_wind = 1;

if if_wind == 0
    
    %%%%%%  The saved ar drone data
    % A = zeros(7); A(1,5) = param.g; A(2,4) = -param.g;
    % A(4,6) = 1; A(5,7) = 1;
    % B = zeros(7,3); B(3,1) = 1/param.m; B(6,2) = 1/param.ixx;
    % B(7,3) = 1/param.iyy;
    % C = zeros(2,7); C(1,4) = 1; C(2,5) = 1;
    % u = [T;tauPhi;tauTheta];
    % y = expData.output.otOrient(1:2,:);
    % x = [xExp(4:8,:); xExp(10:11,:)];
    % save('ardrone_flight_data.mat','A','B','C','u','x','y')
    %%%%%%%%
    
    load('C:\Users\ajithanilmeera\Desktop\matlab\ardrone_flight_data');
    sam_time = t(2)-t(1);
    data_range = 1:500;
    Tt = t(data_range);
    u = u(:,data_range);        % Thrust,torque_roll, torque_pitch
    x = x(:,data_range)';       % [v_x; v_y; v_z; roll; pitch;
    %   ang_vel_roll; ang_vel_pitch]
    y = y(:,data_range)';       % roll; pitch
    ideal_y = y;
    
    %%%%%%%%%%%%%%%%%%%  TODO: COMMENT LATER
    %     u = smoothdata(u','gaussian',5)';
    %     y = y*10;
    
    % Randomly selected model matrices
    while 1
        rng('shuffle')
        ny = size(y,2); nx = 5;
        nu = size(u,1);
        A = -1+2*rand(nx);
        B = -1+2*rand(nx,nu);
        C = -1+2*rand(ny,nx);
        if isstable(ss(A,B,C,0))
            break;
        end
    end
    x = zeros(size(x,1),nx);
    %%%%%%%%%%%%%%%%%%%%%%%
    
else
%     load ardrone2FlightData7_wind2_hPhiDot t ts uLin xLin yLin zPi wPi s A B C;
%     data_range = 1:250;
%     Tt = t(data_range);
%     sam_time = ts;
%     u = uLin(:,data_range);
%     x = xLin(2:3,data_range)';
%     y = yLin(2:2,data_range)';
%     ideal_y = y;
%     A = A(2:3,2:3); B = B(2:3,:); C = C(2,2:3);
    
    %%%%%%%%%% loading the required data %%%%%%%%%%%%%%%%%
    expt_no = 5;
    expt.name  = {'exp_21_WM_1' 'exp_22_WM_1' 'exp_23_WM_1' 'exp_24_WM_2' 'exp_25_WM_2'};
    expt.data_range = [199 899; 1065 1765; 2299 2999; 1043 1743; 135 835];
    load(strcat('C:\Users\ajithanilmeera\Desktop\DEM\drone_data\Matlab_data\',...
                expt.name{expt_no},'.mat'));
    % exp_21_WM_1: data_range = 199:899  (855:1555 bad)
    % exp_22_WM_1: data_range = 1065:1765 (1050:1750 for nice plots of benchmarks)
    % exp_23_WM_1: data_range = 2299:2999
    % exp_24_WM_2: data_range = 1043:1743
    % exp_25_WM_2: data_range = 135:835  (550:1250 special case in ppt for h=e^20)
    
    data_range = expt.data_range(expt_no,1):expt.data_range(expt_no,2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    sam_time = Data_model.dt;
    Tt = (0:size(data_range,2)-1)*sam_time; %Data_model.time(data_range);
    
    if_black_box = 0;
    if if_black_box == 0
        x_selected = [7 10]; % States of the reduced drone model       
        u = Data_model.v(:,data_range);
        x = Data_model.x_measured(x_selected,data_range)';
        
        A = Data_model.sys_c.A(x_selected,x_selected); B = Data_model.sys_c.B(x_selected,:);
        C = eye(2);
        
        % Save full data for model validation
        full_data.t = (0:size(Data_model.time,2)- data_range(1))*sam_time; %Data_model.time(1,data_range(1):end);
        full_data.u = Data_model.v(:,data_range(1):end);
        full_data.y = (C*Data_model.x_measured(x_selected,data_range(1):end))';
        
        y = (C*Data_model.x_measured(x_selected,data_range))';
        ideal_y = y;
    else
        x_selected = [10];%[10], [7 10], [5 7 10], [ 2 5 7 10]; % States of the reduced drone model
        nx = 1;
        x = zeros(size(Tt,2),nx);        
        u = Data_model.v(:,data_range);
        
        % Save full data for model validation
        full_data.t = (0:size(Data_model.time,2)- data_range(1))*sam_time; %Data_model.time(1,data_range(1):end);
        full_data.u = Data_model.v(:,data_range(1):end);
        full_data.y = Data_model.x_measured(x_selected,data_range(1):end)';
        
        y = Data_model.x_measured(x_selected,data_range)';
        ideal_y = y;
        
        % Randomly selected model matrices
        while 1
            rng('shuffle')
            ny = size(y,2); 
            nu = size(u,1);
            A = -1+2*rand(nx);
            B = -1+2*rand(nx,nu);
            C = -1+2*rand(ny,nx);
            if isstable(ss(A,B,C,0))
                break;
            end
        end
    
    end
end

end


function [xtN,sys,Q,R] = EM_for_ss(y,u,dt,nx)
%% Implementation of EM algorithm for discrete linear state space system	
%
% Adapted based on the paper "Using the EM algorithm to estimate the state 
% space model for OMAX" by F.J. Cara, J.Juan, E.Alarcon

[ny,nt] = size(y);
[nu,~] = size(u);

% random initialization of A,B,C matrices
while 1
    rng('shuffle');
%     rng(33);    % for drone data in paper
    Ai = -1+2*rand(nx);
    Bi = -1+2*rand(nx,nu);
    Ci = -1+2*rand(ny,nx);
    if isstable(ss(Ai,Bi,Ci,0))
        break;
    end
end

% sys_i = n4sid(iddata(y',u',dt),nx,n4sidOptions('N4weight','CVA'),...
%              'Ts',dt,'DisturbanceModel','estimate','Form','companion');
% Ai = sys_i.A; Bi = sys_i.B; Ci = sys_i.C;

Qi = eye(nx);
Ri = eye(ny);

max_iter = 75; 
tol = 10^-6;

% x11 = C^{-1}(y_1 - v_1)
m1i = Ci\y(:,1);
P1i = zeros(nx,nx);

A = Ai; B = Bi; C = Ci;
Q = Qi; R = Ri;
m1 = m1i; P1 = P1i;

% log-likelihood values
loglikv = zeros(1,max_iter);

Syy = zeros(ny,ny);
Suu = zeros(nu,nu);
Syu = zeros(ny,nu);
for tt = 1:nt
    Syy = Syy + y(:,tt)*y(:,tt)';
    Suu = Suu + u(:,tt)*u(:,tt)';
    Syu = Syu + y(:,tt)*u(:,tt)';
end

tol1 = 1.0;
iter = 1;
while (iter <= max_iter) && (tol1 > tol)
    
    % Kalman filter
    [xtt,Ptt,xtt1,Ptt1,et,St,Kt,loglik] = KF_for_EM(A,B,C,Q,R,y,u,m1,P1);
    [xtN,PtN,Pt1tN] = KF_smoother_for_EM(A,xtt,Ptt,xtt1,Ptt1);
    
    loglikv(iter) = loglik;
    if iter > 1
        tol1 = abs( (loglikv(iter) - loglikv(iter-1))/loglikv(iter-1) );
    end
    
    % initial values
    Sxx = zeros(nx,nx);
    Sx1x = zeros(nx,nx);
    Syx = zeros(ny,nx);
    Sx1u = zeros(nx,nu);
    Sxu = zeros(nx,nu);
    
    % matrices Sxx, Sx1x, Syx, Sx1u, Sxu, Sx1x1
    for i = 1:nt
        Sxx = Sxx + PtN(:,:,i) + xtN(:,i)*xtN(:,i)';
        Sx1x = Sx1x + Pt1tN(:,:,i) + xtN(:,i+1)*xtN(:,i)';
        Syx = Syx + y(:,i)*xtN(:,i)';
        Sx1u= Sx1u + xtN(:,i+1)*u(:,i)';
        Sxu = Sxu + xtN(:,i)*u(:,i)';
    end
    Sx1x1 = Sxx - (PtN(:,:,1) + xtN(:,1)*xtN(:,1)') + (PtN(:,:,nt+1) + ...
        xtN(:,nt+1)*xtN(:,nt+1)');
    
    % M-step
    % Matrices x0e y P0e
    m1 = xtN(:,1);
    P1 = PtN(:,:,1);
    
    % AB
    AB = [Sx1x Sx1u]/([Sxx Sxu; Sxu' Suu]+exp(-20));
    
    % Matrix A
    A = AB(:,1:nx);
    B = AB(:,nx+1:nx+nu);
    
    % Matrix Q
    M1 = Sx1x*A';
    M2 = Sx1u*B';
    M3 = A*Sxu*B';
    Q = Sx1x1 - M1 - M1' - M2 - M2' + M3 + M3' + A*Sxx*A' + B*Suu*B';
    Q = 1/nt*Q;
    Q = (Q + Q')/2; % to make sure it's a symmetric matrix
    
    % CD
    CD = [Syx Syu]/([Sxx Sxu; Sxu' Suu]+exp(-20));
    C = CD(:,1:nx);
    D = CD(:,nx+1:nx+nu);
    
    % Matrix Re
    M1 = Syx*C';
    M2 = Syu*D';
    M3 = C*Sxu*D';
    R = Syy - M1' - M1 - M2 - M2' + M3 + M3' + C*Sxx*C' + D*Suu*D';
    R = 1/nt*R;
    R = (R + R')/2 ;% to make sure it's a symmetric matrix
    
%     fprintf( "Iter %d, LogLik = %f, tol = %f \n",iter,loglik,tol1);
    iter = iter + 1;
end

sys = ss(A,B,C,D,dt);
loglikv = loglikv(1:iter-1);

% Akaike Information Criterion
P = ny*nx + nx*nx + nx*(nx+1)/2 + ny*(ny+1)/2;
aic = -2*loglikv(end) + 2*P;

% remove the t+dt state prediction from x before returning
xtN(:,end) = [];

end


function [xtt,Ptt,xtt1,Ptt1,et,St,Kt,loglik] = KF_for_EM(A,B,C,Q,R,y,u,x10,P10)
%% Implementation of Kalman Filter for discrete linear state space system
%  for EM algorithm	

[ny,nt] = size(y);
nx = size(A,1);

% allocation
xtt = zeros(nx,nt);
Ptt = zeros(nx,nx,nt);
xtt1 = zeros(nx,nt+1);
Ptt1 = zeros(nx,nx,nt+1);
et = zeros(ny,nt);
St = zeros(ny,ny,nt);
Kt = zeros(nx,ny,nt);
loglik = 0.0;

% Filter
xtt1(:,1) = x10;
Ptt1(:,:,1) = P10;

nt = size(y,2);
for i = 1:nt
    
    et(:,i) = y(:,i) - C*xtt1(:,i) ;
    St(:,:,i) = C*Ptt1(:,:,i)*C' + R; % et variance
   
    % Kalman gain
    Stinv = eye(ny,ny)/(St(:,:,i) + exp(-20)); % numerically preferible to Stinv=inv(St)
    Kt(:,:,i) = Ptt1(:,:,i)*C'*Stinv; % kalman gain
    
    % filtered values
    xtt(:,i) = xtt1(:,i) + Kt(:,:,i)*et(:,i);
    Ptt(:,:,i) = (eye(nx) - Kt(:,:,i)*C)*Ptt1(:,:,i);
    
    % one-step ahead prediction
    xtt1(:,i+1) = A*xtt(:,i) + B*u(:,i);
    Ptt1(:,:,i+1) = A*Ptt(:,:,i)*A' + Q;
    
    % likelihood
    l0 = et(:,i)'*Stinv*et(:,i) ;
    loglik = loglik + log(det(St(:,:,i))) + l0(1);
    
end

loglik =  - ny*nt/2*log(2*pi) - 0.5*loglik;

end

function [xtN,PtN,Pt1tN] = KF_smoother_for_EM(A,xtt,Ptt,xtt1,Ptt1)
%% Implementation of Kalman Smoother for discrete linear state space system
%  for EM algorithm	
    
	[nx,nt] = size(xtt);

	xtN = zeros(nx,nt+1);
	PtN = zeros(nx,nx,nt+1);
	Pt1tN = zeros(nx,nx,nt);

	% values for t = nt+1
	xtN(:,nt+1) = xtt1(:,nt+1);
	PtN(:,:,nt+1) = Ptt1(:,:,nt+1);

	% values for t=nt
	xtN(:,nt) = xtt(:,nt);
	PtN(:,:,nt) = Ptt(:,:,nt);
	Pt1tN(:,:,nt) = A*Ptt(:,:,nt);

	% smother
	for i = nt-1:-1:1
		% Kalman Smoother matrix J
		Jt = (Ptt(:,:,i)*A') / Ptt1(:,:,i+1);

		xtN(:,i) = xtt(:,i) + Jt*( xtN(:,i+1) - xtt1(:,i+1) );
		PtN(:,:,i) = Ptt(:,:,i) + Jt*( PtN(:,:,i+1) - Ptt1(:,:,i+1) )*Jt';
		Pt1tN(:,:,i) = PtN(:,:,i+1)*Jt';
	end

	
    end

function x = kalman_discrete(A,B,C,process_y,P_noise_z,P_noise_w,...
        sam_time,t,real_cause,if_causes)

Bd = (expm(A*sam_time)-eye(size(A)))*pinv(A)*B;

nx = size(A,1);
nt = size(t,2);
x_prev = zeros(nx,1);
Q = inv(P_noise_w);
R = inv(P_noise_z);
x  = zeros(nx,nt);         % EKF estimate of the mean of states
Q  = Q + expm(A)*B*exp(0)*B'*expm(A)';  % EKF process noise variance.
P  = {pinv(full(C'*R*C))};       % EKF conditional covariance of states
% P ={zeros(nx)};
for i = 2:nt
    
    % PREDICTION STEP:
    %----------------------------------------------------------------------
    xPred    = expm(A*sam_time)*x_prev + if_causes*Bd*real_cause(:,i);     % zero causes
    Jx       = expm(A);
    PPred    = Q + Jx*P{i-1}*Jx';
%     PPred =eye(2);
    
    % CORRECTION STEP:
    %----------------------------------------------------------------------
    yPred    = C*xPred;
    Jy       = C;
    K        = PPred*Jy'*inv(R + Jy*PPred*Jy');
    x_prev   = xPred + K*(process_y(i,:)' - yPred);
    x(:,i)   = x_prev;
    P{i}     = PPred - K*Jy*PPred;
%      P{i} =eye(2);
%     plot(i,A*x(:,i) + C'*P_noise_z*(process_y(i,:)' - C*x(:,i)),'r.'); hold on;
%     P{i}
    % report
    %----------------------------------------------------------------------
%     fprintf('EKF: time-step = %i : %i\n',i,nt);
    
end

% figure; plot(x')

end

function kalmfv_x = KF_causes(A,B,C,ny,sam_time,P_brain_w,P_brain_z,...
    real_cause,process_y,t,nx,nu)

%%% General Kalman filter with causes
disc_sys = c2d(ss(A,B,C,zeros(ny,nu)),sam_time);
[~,~,G1] = dare(disc_sys.A',disc_sys.C',inv(P_brain_w),inv(P_brain_z));

Akal = (disc_sys.A-G1'*disc_sys.C);
Bkal = [disc_sys.B G1'];
ukal = [real_cause; process_y'];

M3 = ss(Akal,Bkal,disc_sys.C,0,sam_time);
[~,~,kalmfv_x] = lsim(M3,ukal,t,zeros(nx,1));
kalmfv_x = kalmfv_x';

%%% Friston's Kalman Filter with causes
% if_causes = 1;                   
% kalmfv_x = kalman_discrete(A,B,C,process_y,P_brain_z,P_brain_w,...
%                            sam_time,t,real_cause,if_causes);

end

function [noise_w, noise_z] = make_noise(s_real,P_noise_w,P_noise_z,...
                                         Tt,sam_time,nx,ny,nt)

K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;
rng(12);
noise_z = sqrtm(inv(P_noise_z))*randn(ny,nt)*K;
noise_w = sqrtm(inv(P_noise_w))*randn(nx,nt)*K*sam_time;

end

function s_est = estimate_smoothness(noise_z,Tt)

% s_incr = .005;
% corr_s = zeros(1,round(1/s_incr));
% for i = s_incr:s_incr:1
% corr_s(1,round(i/s_incr)) = find_s(i,noise_z,Tt);
% end
% figure(1); plot(s_incr:s_incr:1,corr_s,'r-');
% [~,min_i] = min(corr_s); 
% s_est = min_i*s_incr

% options = optimset('Display','iter');
func = @(k) find_s(k,noise_z,Tt);
% [s_est, ~] = fminbnd(func, 0,1)

% options = optimoptions('ga','PopulationSize',8,...
%     'MaxGenerations',15,'CrossoverFraction',.8);
% s_est = ga(func,1,[],[],[],[],0.01,1,[],[],options)

% options = optimoptions('simulannealbnd','Display','iter',...
%     'MaxFunctionEvaluations',100,'HybridFcn',@fmincon);
% s_est = simulannealbnd(func,50,0.01,1,options)

options = optimoptions('particleswarm','SwarmSize',50,...
            'HybridFcn',@fmincon,'MaxIterations',4,'Display','iter');
s_est = particleswarm(func,1,0.01,1,options)

% K  = toeplitz(exp(-Tt.^2/(2*s_est^2)));
% K  = diag(1./sqrt(diag(K*K')))*K;
% autocorr(noise_z*pinv(K),'NumLags',50)

end

function [W0,V0y,V0v] = precision_matrix(s_brain,P_brain_w,...
                                         P_brain_z,sigma_v,p,d,nu) 

if p>d, pp = p; else pp = d; end

% Create the smoothness matrix
k          = 0:pp;
x          = sqrt(2)*s_brain;
r(1 + 2*k) = cumprod(1 - 2*k)./(x.^(2*k));
  
Cov     = [];
for i = 1:pp+1;
    Cov = [Cov; r([1:pp+1] + i - 1)];
    r = -r;
end

Cov_inv = pinv(Cov);

W0 = kron(Cov_inv(1:p+1,1:p+1),P_brain_w);
V0y = kron(Cov_inv(1:p+1,1:p+1),P_brain_z);
V0v = kron(Cov_inv(1:d+1,1:d+1),eye(nu)/sigma_v^2);

end

function Y_embed = generalized_process(process_y,prior_cause,t,sam_time,ny,nu,p,d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Embedding higher orders in the process output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nt = size(t,2);
E_y = [  1   0       0      0        0     0        0;
        [1 -1 0 0 0 0 0]/sam_time;
        [1 -2 1 0 0 0 0]/sam_time^2;
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0]]';

Y_embed = zeros(ny*(p+1)+nu*(d+1),nt); %.5*ones(d+1,1)
for i = 1:nt
    if i>p+1 && i<nt-p-1   % Use this section for low sampling time
        Y_embed(:,i) = [embed_Y(process_y',p+1,t(i),sam_time); ...
                       -embed_Y(prior_cause,d+1,t(i),sam_time)];%zeros(d+1,1)];
    else
        Y_embed(1:ny,i) = process_y(i,:)';
        Y_embed(ny*(p+1)+1:ny*(p+1)+nu,i) = -prior_cause(:,i);
    end 
%     Y_embed(1:ny,i) = process_y(i,:)'; % No generalized output
%     Y_embed(:,i) = [embed_Y(process_y',p+1,t(i),sam_time); zeros(d+1,1)];
end
% figure; plot(Y_embed')

% for i = 1:nt
%    if(i > p) 
%        Y_embed(:,i) = [reshape(process_y(i:-1:i-p,:)'*E_y,[],1); zeros(d+1,1)];
%    else 
%        Y_embed(:,i) = [reshape([process_y(i:-1:1,:)' ...
%            repmat(process_y(1,:)',1,p+1-i)]*E_y,[],1); zeros(d+1,1)];
% %        Y_embed(:,i) = zeros((p+1)*4+(d+1),1);
% 
%    end 
% end

end

function [DEM_t,DEM_x] = D_step(A,B,C,dE,Y_embed,V0,W0,Pp,At,Bt,Da,Dv,...
                                nx,ny,nu,nt,p,d,t,sam_time,if_predict_y,if_MF_X)
                          
T = toeplitz(zeros(1,p+1),[0 1 zeros(1,p-1)]);
if p==0; T=0;end
Dy = kron(T,eye(ny));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamic Expectation Maximization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k1 = [kron(eye(p+1),C') zeros(nx*(p+1),nu*(d+1))];
k2 = (At-Da)';
k3 = [zeros(nu*(d+1),ny*(p+1)) -eye(nu*(d+1))];
k4 = kron(eye(p+1,d+1),B)';

D_A = Da-At;

% Ct = kron(eye(p+1),C);
% kkk = sdpvar(1,1);
% kkk1 = sdpvar(1,1);
% AAA = [Da-kkk*D_A'*W0*D_A-kkk*Ct'*V0(1:ny*(p+1),1:ny*(p+1))*Ct kkk*D_A'*W0*Bt;...
%        kkk1*Bt'*W0*D_A                  Dv-kkk1*Bt'*W0*Bt-kkk1*V0(ny*(p+1)+1:end,ny*(p+1)+1:end)];
% 
% optimize([eig(AAA)<=0,kkk>=1, kkk1>=1])
% kkk = value(kkk)
% kkk1 = value(kkk1)
% 
% AAA1 = [(-eye(nx*(p+1))+kkk*D_A'*W0)*D_A Bt-kkk*D_A'*W0*Bt;...
%          -kkk1*Bt'*W0*D_A kkk1*(Bt'*W0*Bt+V0(ny*(p+1)+1:end,ny*(p+1)+1:end))];
% [~,dddd] = chol(AAA1)     

A1 = (k1*V0*blkdiag(kron(eye(p+1),-C),eye((d+1)*nu))+ ...
      k2*W0*[Da-At -Bt]) + [Da zeros((p+1)*nx,(d+1)*nu)];
A2 = ((k3*V0*blkdiag(kron(eye(p+1),-C),eye((d+1)*nu))+ ...
      k4*W0*[Da-At -Bt])+ [zeros((d+1)*nu,(p+1)*nx) Dv]);
A3 = zeros(ny*(p+1));
A4 = Dv;

B1 = k1*V0;
B2 = k3*V0;
B3 = [Dy zeros(ny*(p+1),nu*(d+1))];
B4 = zeros(nu*(d+1),size(B1,2));

% Ad = expm([A1; A2]*sam_time);
% Bd = [A1; A2]\(Ad-eye(size(Ad)))*[B1;B2];
% state_sp = ss([A1;zeros(size(A2))], [B1;zeros(size(B2))], zeros(1,17), zeros(1,31));
if if_predict_y
    state_sp = ss(blkdiag([A1;A2],A3,A4), [B1;B2;B3;B4], ...
        zeros(1,(nx+ny)*(p+1)+nu*(d+1)*2), zeros(1,ny*(p+1)+nu*(d+1)));
else
    state_sp = ss([A1;A2], [B1;B2], ...
        zeros(1,(nx)*(p+1)+nu*(d+1)), zeros(1,ny*(p+1)+nu*(d+1)));
end
state_sp = c2d(state_sp,sam_time,'zoh');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\D_step_Y_generalised.mat');
% [~,DEM_t,DEM_x] = lsim(state_sp,Y_embed,t,zeros(17,1),'zoh');
% figure; plot(DEM_x(:,[1:2 15])); hold on; plot(process_x);

% fprintf('\nStable?? %d\n',isstable(state_sp));

DEM_xx = zeros(nt,size(state_sp.C,2));
Vuu = -[k1 k2; k3 k4]*blkdiag(V0,W0)*[k1 k2; k3 k4]';
Vuy = [k1 k2; k3 k4]*blkdiag(V0,W0)*...
                [kron(eye(p+1),eye(ny)); zeros(nx*(p+1)+nu*(d+1),ny*(p+1))];
Vuv = [k1 k2; k3 k4]*blkdiag(V0,W0)*...
                [zeros(ny*(p+1),nu*(d+1)); -eye(nu*(d+1)); zeros(size(W0,1),nu*(d+1))];

dfdu = [Vuu Vuy Vuv; ...
        zeros(size(Dy,1),size([Vuu Vuy Vuv],2));...
        zeros(size(Dv,1),size([Vuu Vuy Vuv],2))]  + blkdiag(Da,Dv,Dy,Dv);
    
    %     figure; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% delete this line
    
    if if_predict_y
        for i = 2:nt
            q = [DEM_xx(i-1,1:nx*(p+1)+nu*(d+1)) Y_embed(1:ny*(p+1)+nu*(d+1),i-1)'];
            f = blkdiag([A1; A2],A3,A4)*q' + [B1;B2;B3;B4]*Y_embed(:,i-1);
            DEM_xx(i,:) = q + ((expm(dfdu*sam_time)-eye(size(f,1)))*pinv(dfdu)*f)';
        end
        DEM_xx = DEM_xx(:,1:nx*(p+1)+nu*(d+1));
    else
        for i = 2:nt
            q = DEM_xx(i-1,1:nx*(p+1)+nu*(d+1)); 
            [dWdX,dWdXX] = mean_field_state(q',dE,p,d,nx,ny,nu,Pp,V0(1:ny*(p+1),1:ny*(p+1)),W0);
            f = [A1; A2]*q' + [B1;B2]*Y_embed(:,i-1) + if_MF_X*dWdX;
            dfdu = Vuu + if_MF_X*dWdXX;
            DEM_xx(i,:) = q + ((expm(dfdu*sam_time)-eye(size(f,1)))*pinv(dfdu)*f)';

            
%             DEM_xx(i,:)=(state_sp.A*DEM_xx(i-1,:)' + state_sp.B*Y_embed(:,i))';
        end
        DEM_xx(1:nt-1,:) = DEM_xx(2:nt,:);
    end

% plot(1:320,DEM_xx(:,1:nx)','r'); hold on;

DEM_x = DEM_xx;
DEM_t = t;

end

function DEMv_x = D_step_causes(A,D_A,B,Da,Bt,Ct,V0y,W0,Y_embed,real_cause,...
    t,sam_time,nt,ny,nu,p_brain,d_brain)

state_sp_v = ss(Da - Ct'*V0y*Ct - D_A'*W0*D_A, ...
                [Ct'*V0y D_A'*W0*Bt], zeros(1,size(Da,2)), ...
                zeros(1,(p_brain+1)*ny+(d_brain+1)*nu));
state_sp_v = c2d(state_sp_v,sam_time,'zoh');

% Embed the known causes
V_embed = zeros(nu*(d_brain+1),nt);
% V_embed(1,:) = real_cause;
for i = 1:nt
    V_embed(:,i) = embed_Y(real_cause,d_brain+1,t(i),sam_time);
end
Y_embed(ny*(p_brain+1)+1:end,:) = V_embed;

% Perform state estimation
DEMv_x = zeros(nt,size(state_sp_v.C,2));
% figure; 
for i = 2:nt
    DEMv_x(i,:)=(state_sp_v.A*DEMv_x(i-1,:)' + state_sp_v.B*Y_embed(:,i))';
%     plot(i,A'*W0*(A*DEMv_x(i,1:2)'+B*real_cause(i)),'r.'); hold on;
%     plot(i,Ct'*V0y*(Y_embed(1:4,i)-Ct*DEMv_x(i,1:2)'),'b.'); hold on;
end

end


function X = spm_inv(A,TOL)
% inverse for ill-conditioned matrices
% FORMAT X = spm_inv(A,TOL)
%
% A   - matrix
% X   - inverse
%
% TOL - tolerance: default = max(eps(norm(A,'inf'))*max(m,n),exp(-32))
%
% This routine simply adds a small diagonal matrix to A and calls inv.m
%__________________________________________________________________________
% Copyright (C) 2008 Wellcome Trust Centre for Neuroimaging
 
% Karl Friston
% $Id: spm_inv.m 7143 2017-07-29 18:50:38Z karl $
 
% check A 
%--------------------------------------------------------------------------
[m,n] = size(A);
if isempty(A), X = sparse(n,m); return, end
 
% tolerance
%--------------------------------------------------------------------------
if nargin == 1
    TOL  = max(eps(norm(A,'inf'))*max(m,n),exp(-32));
end

% inverse
%--------------------------------------------------------------------------
X     = inv(A + speye(m,n)*TOL);
end

function print_results(SSE,j,k,if_UIO,if_cause)

if if_UIO == 1
    fprintf('DEM_x, UIO_x, KF_x = ');
    disp(vpa([SSE.DEM.x(j,k) SSE.UIO.x(j,k) SSE.kalman.x(j,k)],4))
    if if_cause == 1
        fprintf('DEMv_x, KFv_x = ');
        disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
    end
    fprintf('DEM_v, UIO_v = ');
    disp(vpa([SSE.DEM.v(j,k) SSE.UIO.v(j,k)],4));
else
    fprintf('DEM_x, KF_x, DEM_v = ');
    disp(vpa([SSE.DEM.x(j,k) SSE.kalman.x(j,k) SSE.DEM.v(j,k)],4))
    if if_cause == 1
        fprintf('DEMv_x, KFv_x = ');
        disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
    end
    
end
end

function writeasGIF(fig,para,name)
%% Save the figure as gif

    filename = strcat('C:\Users\ajithanilmeera\Desktop\DEM\',name);

    % Capture the plot as an image
    frame = getframe(fig);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if para == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append');
    end
end

function plot_results(output,model,brain,if_UIO,if_cause)
%% Plot the results

%
color = [ 0         0.4470    0.7410;    0.8500    0.3250    0.0980;
          0.9290    0.6940    0.1250;    0.4940    0.1840    0.5560;
          0.4660    0.6740    0.1880;    0.3010    0.7450    0.9330;
          0.6350    0.0780    0.1840];

% plot DEM with and without causes
if if_cause == 1
    figure;
    fig1 = plot(output.DEM_t,output.DEMv_x(:,1:brain.nx),'-.','Color',color(1,:)); hold on;
    fig2 = plot(output.DEM_t,output.kalmfv_x,'--','Color',color(2,:)); hold on;
    fig3 = plot(output.DEM_t,model.ideal_x,'-','Color',color(4,:));
    legend([fig1(1),fig2(1),fig3(1)],...
        {'DEMv states','KFv states','Ideal states'});
    xlabel time(s);
end

% plot state estimates and true states
figure; subplot(2,1,1);
fig1 = plot(model.t,model.ideal_x,'-','Color',color(1,:)); hold on;
fig2 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(2,:)); hold on;
fig3 = plot(model.t,output.kalman_x','-.','Color',color(4,:)); hold on;
if if_UIO == 1
    fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(5,:)); hold on;
    legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states','DEM states',...
        'Kalman estimate','UIO estimate'});    
else
    legend([fig1(1),fig2(1),fig3(1)],{'Ideal states','DEM states','Kalman estimate'});
end

% plot causal state estimate and true cause
subplot(2,1,2)
plot(output.DEM_t,output.DEM_x(:,brain.nx*(brain.p+1)+1),'--'); hold on;
plot(model.t,model.real_cause','-','LineWidth',1); hold on;
if if_UIO == 1
    plot(model.t,output.UIO_v_est,'-.'); hold on;
    legend('DEM cause estimate','Ideal causal state','UIO cause estimate');
else
    legend('DEM cause estimate','Ideal causal state');    
end

% plot kalman state estimates and true states
% figure; 
% fig1 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(1,:)); hold on;
% fig2 = plot(model.t,output.kalman_x','-.','Color',color(2,:)); hold on;
% fig3 = plot(model.t,model.ideal_x,'-','Color',color(4,:));
% legend([fig1(1),fig2(1),fig3(1)],...
%     {'DEM states','Kalman states','Ideal states'});
% xlabel time(s)
end

function brain = plot_FE_manifold(output,brain,model,if_MF_p,para)
    %%%%%%%%%%%%%%%%%%%%%%%%% Plotting Free energy manifold %%%%%%%%%%
        saved_brain = brain;
        sim_A = brain.A + (-9:.5:9); sim_B = brain.B + (-9:.5:9); 
        F_bar = zeros(length(sim_A),length(sim_B));
        for pp1 = 1:length(sim_A)
            for pp2 = 1:length(sim_B)
                brain = saved_brain;
                brain.A = sim_A(pp1) ;
                brain.B = sim_B(pp2) ;
                brain.theta = [reshape(brain.A',[],1); reshape(brain.B',[],1); ...
                    reshape(brain.C',[],1)];
                brain = set_brain(brain);
                brain = update_parameters(output.DEM_x,brain,if_MF_p);
                F_bar(pp1,pp2) = brain.FreeE(end);
            end
        end
        figure(342); [xxx,yyy] = meshgrid(sim_A,sim_B); hold on;
        surf(xxx,yyy,F_bar'); hold on; alpha 1;
        [aa,bb] = find(F_bar==max(max(F_bar)));
        hold on; plot3(sim_A(aa),sim_B(bb),F_bar(aa,bb),'r*'); 
        view(30,30); 
        
               
        % Reset the brain to old saved brain
        brain = update_parameters(output.DEM_x,saved_brain,if_MF_p);
        brain = set_brain(brain);
        figure(342); hold on; 
        plot3(saved_brain.A,saved_brain.B,brain.FreeE(end),'b*'); 
        ztickformat('%.2f'); set(gca,'fontsize',12); 
        xlabel('A'); ylabel('B'); zlabel('$\bar{F}_o$','Interpreter','latex');
        
        % Plot parameter convergence and free energy maximization
        figure(343); hold on; plot(sim_A,F_bar(:,find(sim_B==saved_brain.B)),'-','color','#D95319');
        hold on; plot(sim_B,F_bar(find(sim_A==saved_brain.A),:),'-','color','#0072BD');
        hold on; plot(saved_brain.A,brain.FreeE(end),'o','Linewidth',2,'color','#D95319');        
        hold on; plot(saved_brain.B,brain.FreeE(end),'o','Linewidth',2,'color','#0072BD');
%         hold on; plot([model.A model.A],get(gca).YLim,'--','Linewidth',2,'color','#EDB120');
%         hold on; plot([model.B model.B],get(gca).YLim,'--','Linewidth',2,'color','#7E2F8E');        
%         
        legend({'$\bar{F}_o$ v/s A','$\bar{F}_o$ v/s B','$A_{est}$',...
                '$B_{est}$','real A','real B'},'Interpreter','latex',...
                'NumColumns',2,'Orientation','horizontal');
        text(saved_brain.A-4,brain.FreeE(end),strcat('i=',int2str(para),'\rightarrow '))     
        text(saved_brain.B+.5,brain.FreeE(end),strcat('\leftarrow i=',int2str(para)))     
        xlabel('Parameter A, B'); ylabel('$\bar{F}_o$','Interpreter','latex');
        ytickformat('%.2f'); set(gca,'fontsize',12); 

end

function plot_estimation(model,brain,output,para)
    fig5 = figure(5); clf; 
    set(gcf,'color','w'); subplot(2,3,1);
    mean_2 = output.DEM_x(:,1:brain.nx);
    Pu = [brain.Pu(1:brain.nx,1:brain.nx) brain.Pu(1:brain.nx,...
        brain.nx*(brain.p+1)+1); brain.Pu(brain.nx*(brain.p+1)+1,1:brain.nx) ...
            brain.Pu(brain.nx*(brain.p+1)+1,brain.nx*(brain.p+1)+1) ];
    std_2  = 1./sqrt(diag(brain.Pu(1:brain.nx,1:brain.nx)))';
    fill([model.t fliplr(model.t)],...
         [full(mean_2 + std_2)' fliplr(full(mean_2 - std_2)')],...
                 [1 1 1]*.8,'EdgeColor',[1 1 1]*.8)
    hold on; plot(model.t,output.DEM_x(:,1:brain.nx),'b'); hold on; 
    plot(model.t,model.process_x,'r'); title('state estimation'); xlabel('time');
    
    % Plot input estimation
    subplot(2,3,2);
    index_v = brain.nx*(brain.p+1)+1:brain.nx*(brain.p+1)+brain.nu;
    mean_2 = output.DEM_x(:,index_v);
    std_2  = sqrt(1./diag(brain.Pu(index_v,index_v)))';
    fill([model.t fliplr(model.t)],...
         [full(mean_2 + std_2)' fliplr(full(mean_2 - std_2)')],...
                 [1 1 1]*.8,'EdgeColor',[1 1 1]*.8)
    hold on; plot(model.t,mean_2,'b'); hold on; 
    plot(model.t,model.real_cause','r'); title('input estimation'); xlabel('time');

    % Plot parameters
    subplot(2,3,[4,5]); 
    mean_1 = [model.theta,brain.theta];
    std_1  = [0*model.theta,brain.S_theta];
    hBar = bar(mean_1); hold on; 
    errorbar(bsxfun(@plus, hBar(1).XData, [hBar.XOffset]')',mean_1,std_1,'k.')
    ylim([-1.2,1.2]);   
    legend('\theta','\theta est','location','best');
    
    [yyy,~,~] = lsim(ss(brain.A,brain.B,brain.C,[]),...
        output.DEM_x(:,brain.nx*(brain.p+1)+[1:brain.nu])',...
        model.t-model.sam_time,[],'foh');
    subplot(2,3,6);
    plot(model.t,yyy,'b'); hold on; plot(model.t,model.process_y,'r');
    legend('O/p pred');
    title('Rolled out output'); xlabel('time');
    
    % Plot free energy
    subplot(2,3,3); 
    plot(brain.FreeE - brain.FreeE(1)); title('Free energy')
       
    fig11 = figure(11); clf; 
    set(gcf,'color','w'); subplot(2,3,1:2);
    h1 = plot(model.t,(brain.A*output.DEM_x(:,1:brain.nx)'+brain.B*...
        output.DEM_x(:,brain.nx*(brain.p+1)+[1:brain.nu])')','b'); hold on;
    kkk = (brain.Da*output.DEM_x(:,1:brain.nx*(brain.p+1))')';
    h2 = plot(model.t,kkk(:,1:brain.nx),'r'); hold on;
    h3 = plot(model.t,kkk(:,1:brain.nx) - (brain.A*output.DEM_x(:,1:brain.nx)'+brain.B*...
        output.DEM_x(:,brain.nx*(brain.p+1)+[1:brain.nu])')','k:');
    legend([h1(1) h2(1) h3(1)],'AX+BU','DX','noise w');
    title('State predictions: DX and AX+BU'); xlabel('time');
    
    % Plot output prediction
    subplot(2,3,4:5);
    h1 = plot(model.t,(brain.C*output.DEM_x(:,1:brain.nx)')','b'); hold on;
    h2 = plot(model.t,model.process_y,'r'); hold on;
    h3 = plot(model.t,model.process_y - (brain.C*output.DEM_x(:,1:brain.nx)')','k:'); hold on;
    legend([h1(1) h2(1) h3(1)],'CX','Y','noise z');
    title('Output predictions: CX');    xlabel('time');
    
    subplot(2,3,3);     plot(brain.memory.h'); hold on;
    title('Hyperparameter update'); legend('\lambda^z','\lambda^w','location','northwest');
    subplot(2,3,6);     plot(brain.memory.theta'); hold on;
    title('Parameter update');
%     subplot(2,2,2);   plot(brain.memory.h); hold on;
    
    % Evolution of recognition density of parameters over each update
%     figure(777);
%     [xx,yy] = meshgrid(1:size(brain.memory.theta,2),-3:.01:3); 
%     surf(xx,yy,normpdf(yy,brain.memory.theta(1,1:end),...
%         brain.memory.S_theta(1,1:end)),'edgecolor','none');

    
    % Save plot as GIF
%     writeasGIF(fig5,para,'state_input_FE.gif');
%     writeasGIF(fig11,para,'DEM.gif');
    
end

function [process_x, process_y] = generate_data(A,B,C,noise_w,noise_z,...
                                                real_cause,t,sam_time,p,d)
if_GC = 1; % If generalized coordinates should be used in generative process

nx = size(A,1);
ny = size(C,1);
nu = size(B,2);
nt = size(t,2);

if if_GC == 0
    ideal_sp = c2d(ss(A,B,C,zeros(ny,nu)),sam_time,'zoh');
end

process_x = zeros(nx,nt)';
process_y = zeros(ny,nt)';

U.x = zeros(nx,p+1);
U.v = zeros(ny+nu,p+1);
process_y(1,:) = noise_z(:,1);
if if_GC == 1
    
    Dv_j = kron(diag(ones(1,p),1),eye(ny+nu));
    Dx_j = kron(diag(ones(1,p),1),eye(nx));
    D_j = blkdiag(Dv_j,Dx_j,Dv_j,Dx_j);
    dgdv = zeros((p+1)*(ny+nu));
    dgdx = kron(diag(ones(1,p),1),[C; zeros(nu,nx)]);
    dfdv = kron(eye(p+1),[zeros(size(C')) B]);
    dfdx = kron(eye(p+1),A);
    dfdw = kron(eye(p+1),eye(nx));
    J = [dgdv                              dgdx     Dv_j                            zeros(size(Dv_j,1),size(Dx_j,2));...
         dfdv                              dfdx     zeros((p+1)*nx,size(Dv_j,2))    dfdw;...
         zeros((p+1)*[(ny+nu),(ny+nu+nx)])          Dv_j                            zeros(size(Dv_j,1),size(Dx_j,2));...
         zeros((p+1)*[nx,(ny+nu+nx)])      zeros(size(Dx_j,1),size(Dv_j,2))     Dx_j];
    
    %------------- REDUCED ORDER DATA GENERATION [Y,X]-----------
%     AA = [zeros(ny*(p+1)) kron(eye(p+1),C)*Dx_j; zeros(nx*(p+1),ny*(p+1))...
%         kron(eye(p+1),A)];
%     BB = [zeros(ny*(p+1),nu*(p+1)) kron(diag(ones(1,p),1),eye(ny)) ...
%         zeros(ny*(p+1),nx*(p+1)); kron(eye(p+1,p+1),B) zeros(nx*(p+1),ny*(p+1)) ...
%         eye(nx*(p+1))];
%     CC = [1 zeros(1,(p+1)*(ny+nx)-1)];
%     sys = c2d(ss(AA,BB,CC,[]),sam_time,'zoh');
%     yx(:,1) = [noise_z(:,1); zeros((ny+nx)*(p+1)-ny,1)];
    %----------------------------------------------------------
    
    for i=1:nt-1
        % generalized noises and causes
        d_noise_w = embed_Y(noise_w,p+1,t(i),sam_time);
        d_noise_z = embed_Y([noise_z; real_cause],p+1,t(i),sam_time);
        d_noise_w = reshape(d_noise_w,nx,[]);
        d_noise_z = reshape(d_noise_z,ny+nu,[]);
        
        pU.x(:,1)  = U.x(:,1);
        pU.v(:,1)  = [C*U.x(:,1); zeros(nu,1)] + [noise_z(:,i); real_cause(:,i)];
        pU.x(:,2)  = A*U.x(:,1) + [zeros(size(C')) B]*U.v(:,1) + noise_w(:,i);
        for j = 2:p
            pU.v(:,j) = [C*pU.x(:,j); zeros(nu,1)] + d_noise_z(:,j);
            pU.x(:,j+1) = [zeros(size(C')) B]*pU.v(:,j) + A*pU.x(:,j) + ...
                d_noise_w(:,j);
        end
        pU.v(:,p+1) = [C*pU.x(:,p+1); zeros(nu,1)] + d_noise_z(:,p+1);

        qU = [reshape(pU.v,[],1); reshape(pU.x,[],1); ...
            reshape(d_noise_z,[],1); reshape(d_noise_w,[],1)];        
%         qU = expm(J*sam_time)*qU; 
        qU = qU + (expm(J*sam_time) - eye(size(J)))*pinv(J)*D_j*qU;

        %---------------------REDUCED ORDER DATA GENERATION [Y,X]------            
%         u_tilde = embed_Y(real_cause,p+1,t(i),sam_time);
%         z_tilde = embed_Y(noise_z,p+1,t(i),sam_time);
%         w_tilde = embed_Y(noise_w,p+1,t(i),sam_time);
%         
%         x_tilde(:,1)  = yx(ny*(p+1)+(1:nx),i);  % x
%         y_tilde(:,1)  = C*x_tilde(:,1) + noise_z(:,i);
%         x_tilde(:,2)  = A*x_tilde(:,1) + B*real_cause(:,i) + noise_w(:,i);
%         for j = 2:p
%             y_tilde(:,j) = C*x_tilde(:,j) + z_tilde((j-1)*ny+(1:ny));
%             x_tilde(:,j+1) = A*x_tilde(:,j) + B*u_tilde((j-1)*nu+(1:nu)) ...
%                                             + w_tilde((j-1)*nx+(1:nx));
%         end
%         y_tilde(:,p+1) = C*x_tilde(:,p+1)+ z_tilde(p*ny+(1:ny));
%         yx_tilde = [reshape(y_tilde,[],1); reshape(x_tilde,[],1)]; 
%         yx(:,i+1) = sys.A*yx_tilde + sys.B*[u_tilde; z_tilde; w_tilde];
        %-------------------------------------------------------------------

        
        U.v = reshape(qU(1:(p+1)*(ny+nu),1),ny+nu,p+1);
        U.x = reshape(qU((p+1)*(ny+nu)+1:(p+1)*(ny+nu+nx),1),...
            nx,p+1);
        process_y(i+1,:) = U.v(1:ny,1);
        process_x(i+1,:) = U.x(:,1);
    end
    %----------------REDUCED ORDER DATA GENERATION [Y,X]------------
%     process_y = yx(1:ny,:)';
%     process_x = yx(ny*(p+1)+(1:nx),:)';
    %---------------------------------------------------------------
else
    % Generative process without generalized motion on noises
    for i=2:nt
        process_x(i,:) = (ideal_sp.A*process_x(i-1,:)' + ...
            ideal_sp.B*real_cause(:,i-1)+ noise_w(:,i-1))';
        process_y(i-1,:) = (ideal_sp.C*process_x(i-1,:)' + ...
            ideal_sp.D*real_cause(:,i-1)+ noise_z(:,i-1))';
    end
end
if if_GC == 0
    process_y(end,:) = (ideal_sp.C*process_x(end,:)' + ...
        ideal_sp.D*real_cause(:,end)+ noise_z(:,end))';
end

end

function dY = embed_Y(Y,n,t,dt)

    [~, N]  = size(Y);
    T = zeros(n);
    
    s      = round((t)/dt);
    k      = (1:n)  + fix(s - (n + 1)/2);
    x      = s - min(k) + 1;
    y_pad = k<x-1 | k> N-(x-1);% Extra step: truncate innacurate derivative at edges
    i      = k < 1;
    k      = k.*~i + i;
    i      = k > N;
    k      = k.*~i + i*N;


    % Inverse embedding operator (T): cf, Taylor expansion Y(t) <- T*y[:]
    %----------------------------------------------------------------------
    for i = 1:n
        for j = 1:n
            T(i,j) = ((i - x)*dt)^(j - 1)/prod(1:(j - 1));
        end
    end

    % embedding operator: y[:] <- E*Y(t)
    %----------------------------------------------------------------------
    E     = inv(T);
   
    dY      = Y(:,k)*E';
    dY(:,end-sum(y_pad)+1:end) = zeros(size(Y,1),sum(y_pad)); % truncate inaccurate derivatives
    dY = reshape(dY,[],1);
end
