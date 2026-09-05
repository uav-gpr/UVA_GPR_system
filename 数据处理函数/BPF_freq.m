function dataout=BPF_freq(data_freq,label_f,f1,f2,f3,f4)   
% 频率的单位要求是GHz
% bandpass filter 

%正弦函数的平方作为镶边函数
Hf = zeros(length(label_f),1);
for i = 1:length(label_f)
    if (label_f(i)>=f1)&&(label_f(i)<=f2)
        Hf(i) = (sin(pi/2*(label_f(i)-f1)/(f2-f1)))^2;
    elseif (label_f(i)>=f2)&&(label_f(i)<=f3)
        Hf(i) = 1;
    elseif (label_f(i)>=f3)&&(label_f(i)<=f4)
        Hf(i) = (sin(pi/2*(f4-label_f(i))/(f4-f3)))^2;
    end
end
dataout = data_freq.*Hf;
end
