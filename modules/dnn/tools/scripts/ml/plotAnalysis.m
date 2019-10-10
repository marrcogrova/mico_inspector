function [] = plotAnalysis(_lda, _gt)
    [phi, theta] = loadParams(_lda);
    gt = load(_gt);
    drawTetraDocs(theta, gt,'o');
end