scriptDir = fileparts(mfilename("fullpath"));
addpath(scriptDir);

projectBrief = "Develop a MATLAB-centered virtual lab workflow for early-stage nanobody design against a new SARS-CoV-2 variant. The first version should focus on planning, tool selection, and workflow definition rather than executing AlphaFold or Rosetta directly.";

results = runVirtualLabWorkflow(projectBrief, ...
    SaveRoot=fullfile(scriptDir, "workflow_output"), ...
    NumIterations=2, ...
    NumRounds=2, ...
    UseSDK=false, ...
    DryRun=true);

disp("Workflow phases:")
for idx = 1:numel(results.PhaseResults)
    phase = results.PhaseResults(idx);
    fprintf("%s -> %s\n", phase.Name, phase.MergedSummary);
end
