% Generate the figures embedded in RESULTS.md from the trained models.
%
% Run the three example scripts first so that the model files
% parameters_function<name>.mat exist in the robot folders:
%   UR3e/GeneralModelExample.m, UR10e/GeneralModelUR10.m,
%   Franka Panda/GeneralModelFranka.m
%
% Figures are written to the Figures/ folder.

repo = fileparts(mfilename('fullpath'));
addpath(fullfile(repo, 'Library'));
addpath(fullfile(repo, 'UR3e'));
addpath(fullfile(repo, 'UR10e'));
addpath(fullfile(repo, 'Franka Panda'));
outDir = fullfile(repo, 'Figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end

R(1) = struct('tag', 'UR3e',  'model', "ModelUR3e",  'n', 6, ...
    'csv', fullfile(repo, 'UR3e', 'URRobotData_Test.csv'), ...
    'sig', 'Current', 'unit', 'A');
R(2) = struct('tag', 'UR10e', 'model', "ModelUR10e", 'n', 6, ...
    'csv', fullfile(repo, 'UR10e', 'UR10RobotData_Test.csv'), ...
    'sig', 'Current', 'unit', 'A');
R(3) = struct('tag', 'Panda', 'model', "PandaRobot", 'n', 7, ...
    'csv', fullfile(repo, 'Franka Panda', 'FrankaRobotData_Test.csv'), ...
    'sig', 'Torque', 'unit', 'Nm');

for r = 1:numel(R)
    N = R(r).n;
    D = readmatrix(R(r).csv);
    t = D(:,1);
    expq = D(:, 2:N+1);
    expdq = D(:, N+2:2*N+1);
    expddq = D(:, 2*N+2:3*N+1);
    Imeas = D(:, 3*N+2:4*N+1);
    PC = D(:, end);

    [PE, I, e] = Test_Model(R(r).model, expq, expdq, expddq, t, PC);

    % --- Figure 1: total electric power on the test set -----------------
    fig = figure('Position', [100 100 900 420], 'Color', 'w');
    try, fig.Theme = 'light'; catch, end %#ok<TRYNC> (Theme requires R2025a+)
    plot(t, PC, 'LineWidth', 1.0)
    hold on
    plot(t, PE, 'LineWidth', 1.0)
    grid on
    xlabel('Time [s]')
    ylabel('Electric power [W]')
    legend('Measured', 'Estimated', 'Location', 'best')
    title(sprintf('%s — test set power estimation (RMSE = %.2f W, R^2 = %.3f)', ...
        R(r).tag, e(1), e(3)))
    exportgraphics(fig, fullfile(outDir, [R(r).tag '_power_test.png']), 'Resolution', 150);

    % --- Figure 2: measured vs estimated power (scatter) ----------------
    fig = figure('Position', [100 100 460 420], 'Color', 'w');
    try, fig.Theme = 'light'; catch, end %#ok<TRYNC>
    plot(PC, PE, '.', 'MarkerSize', 4)
    hold on
    lims = [min([PC; PE]) max([PC; PE])];
    plot(lims, lims, 'k--', 'LineWidth', 1.0)
    grid on
    axis equal
    xlim(lims); ylim(lims)
    xlabel('Measured power [W]')
    ylabel('Estimated power [W]')
    title(sprintf('%s — measured vs estimated', R(r).tag))
    exportgraphics(fig, fullfile(outDir, [R(r).tag '_power_scatter.png']), 'Resolution', 150);

    % --- Figure 3: per-joint current/torque estimation ------------------
    fig = figure('Position', [100 100 1000 200*ceil(N/2)], 'Color', 'w');
    try, fig.Theme = 'light'; catch, end %#ok<TRYNC>
    tl = tiledlayout(ceil(N/2), 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:N
        nexttile
        plot(t, Imeas(:,j), 'LineWidth', 0.8)
        hold on
        plot(t, I(:,j), 'LineWidth', 0.8)
        grid on
        title(sprintf('Joint %d', j))
        if j > N-2, xlabel('Time [s]'); end
        ylabel(sprintf('%s [%s]', R(r).sig, R(r).unit))
    end
    lg = legend('Measured', 'Estimated', 'Orientation', 'horizontal');
    lg.Layout.Tile = 'north';
    title(tl, sprintf('%s — test set joint %s estimation', R(r).tag, lower(R(r).sig)))
    exportgraphics(fig, fullfile(outDir, [R(r).tag '_joints_test.png']), 'Resolution', 150);

    fprintf('%s figures done (RMSE %.2f W, R^2 %.3f)\n', R(r).tag, e(1), e(3));
end
