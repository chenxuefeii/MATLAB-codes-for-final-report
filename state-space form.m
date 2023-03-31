[x1,u1,y1,dx1] = trim('linearise_model2',[30 0 0]',0);
[A1,B1,C1,D1] = linmod('linearise_model2',x1,u1,y1);
sys0 = ss(A1,B1,C1,D1);
tf(sys0)
con1 = [B1 A1*B1 A1*A1*B1];% rank = 3, controllable
rank(con1);
