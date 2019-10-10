%Single performan Analysis
paramFile = 'new_lda.txt';
gtTrainFile = 'iters_varying/gt_(cam_cougar_moto_face).txt';
resultFile = 'results.txt'
gtCvFile = 'iters_varying/gt_results.txt';

[phi, theta] = loadParams(paramFile);
gt = load(gtTrainFile);
results = loadResults(resultFile);
gtCv = load(gtCvFile);

figure();
drawTetraDocs(theta, gt,'o');
hold on;
drawTetraDocs(results, gtCv, '*');

% figure();
% bar(phi);
