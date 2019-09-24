
trainWords = [100,200,300,500,700];
alphas = [0.1,0.3,0.7,1,5,10,20];
betas = [0.1,0.3,0.7,1,5,10,20];

gtFile = "/home/bardo91/programming/A-DATASETS/LDA_BOW_SVM/cam_cougar_moto_face/train/gt.txt";
pathldas = "/home/bardo91/programming/rgbd_tools/build/";
for i = 1:length(trainWords)
  figure();
  for j = 1:length(alphas)
    for k = 1:length(betas)
        subplot(length(alphas), length(betas), j*length(alphas)+k)
        title(strcat("a",mat2str(alphas(j))," b",mat2str(betas(k))," w",mat2str(trainWords(i))))
        filename = strcat("model_a",mat2str(int32(alphas(j)*10)),"_b",mat2str(int32(betas(k)*10)),"_w",mat2str(trainWords(i)));
        plotAnalysis(strcat(pathldas,filename), gtFile);
    end     
  end   
end 

