%Single performan Analysis


% paramBrisk = 'descriptor_comp/brisk/lda.txt';
% paramOrb500 = 'descriptor_comp/orb500/lda.txt';
% paramOrb1000 = 'descriptor_comp/orb1000/lda.txt';
% paramOrb5000 = 'descriptor_comp/orb5000/lda.txt';
% paramOrb500Freak = 'descriptor_comp/orb500+Freak/lda.txt';
% paramOrb1000Freak = 'descriptor_comp/orb1000+Freak/lda.txt';
% paramOrb5000Freak = 'descriptor_comp/orb5000+Freak/lda.txt';
% paramSift = 'descriptor_comp/sift/lda.txt';

folders = { 'descriptor_comp/brisk/', ...
            'descriptor_comp/orb500/', ...
            'descriptor_comp/orb1000/', ...
            'descriptor_comp/orb5000/', ...
            'descriptor_comp/orb500+Freak/', ...
            'descriptor_comp/orb1000+Freak/', ...
            'descriptor_comp/orb5000+Freak/', ...
            'descriptor_comp/sift/'};

for i=1:length(folders)
   plotAnalysis(folders{i}); 
end