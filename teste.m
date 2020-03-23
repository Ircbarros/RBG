    r=0.25;
    c=0.5;
    Lamda_L=20;
    Lamda_R=20;
    w_L=0.1; 
    w_R=4*w_L;
    t=0:0.1:6.28;
    eita=(r/2)*[1-Lamda_L,1-Lamda_R ;(Lamda_L-1)/c,(1-Lamda_R)/c]*[w_L ; w_R];
    Vm_x = eita(1,:)
    wz=eita(2,:);
    for i=1:63%%calculation of theta
      theta(i)=i*wz;
    end
    Vox=Vm_x*cos(theta);
    Voy=Vm_x*sin(theta);
    y=zeros(1,63);
    x=zeros(1,63);
    x=Vox.*t;
    y=Vox.*t;
    plot(x,y)