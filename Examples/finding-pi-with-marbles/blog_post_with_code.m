%[text] # Using Claude Code and MATLAB to Search for a Pi Estimator
%[text] A recent [preprint](https://arxiv.org/abs/2602.14487) describes a way to calculate $\pi$ using coin flips. Other physical estimates, such as throwing darts at a square and counting how many land inside a circle, are familiar but slow and not ideal for a classroom. I wanted a simple random process that a class could run by hand, then test and prove with MATLAB.
%[text] I spent an afternoon with Claude Code connected to a live MATLAB session using the [MATLAB Agentic Toolkit](https://www.mathworks.com/products/matlab-agentic-toolkit.html) and MCP server. The goal was to use Claude as an experimental partner: propose candidate processes, write and run MATLAB simulations, and check the algebra when a candidate looked promising.
%%
%[text] ## The Workflow: Claude Code + MATLAB MCP
%[text] Claude Code is a command-line AI agent that can read and write files, run commands, and call terminal tools. Here, it wrote MATLAB code, ran it through the MCP connection, inspected the results, and revised the next attempt in the same loop.
%[text] The loop went like this:
%[text] - I described the main goal and a few starting approaches to consider: stopping rules, Poisson arrivals, and variants of coin-flip or birthday-problem processes. Claude then suggested some additional families of random processes to try.
%[text] - For each candidate, Claude wrote a MATLAB simulation, ran it through the MCP server, and analyzed the results.
%[text] - We dropped ideas that did not converge to $\\pi${"editStyle":"visual"}, converged too slowly, or appeared to have systematic bias.
%[text] - For the methods that survived numerically, I asked Claude to use the [Symbolic Math Toolbox](https://www.mathworks.com/products/symbolic.html)™ to check whether there was a simple closed-form proof.
%[text] This made short feedback loops possible: test an idea, inspect the result, then keep or discard it without hand-writing each experiment.
%%
%[text] ## The Marble Bag Stopping Rule
%[text] We started by testing many ideas. A birthday-problem approach gave $\pi$ only approximately, with bias that depended on the number of possible birthdays. Counting ties in coin flips was too noisy to be useful. An early marble-bag version that added one blue marble per round instead of two produced a value close to $\pi$, but the exact result was $(8 log(2) + 4)/3$.
%[text] The useful idea came from hazard-rate processes: random stopping games where the probability of stopping changes each round. After testing several rules in MATLAB, we found one where the probability of stopping on round k, assuming the game has reached that round, is 1/(2k). The probability of surviving the first n rounds becomes
%[text] $S(n)=\\frac{\\binom{2n}{n}}{4^n}$,$$
%[text] which involves the central binomial coefficients. Those same coefficients appear in the Taylor series for $\arcsin(x)$, and when that series is evaluated at $x=1$, $\arcsin(1)=\pi/2$. That was the first evidence that the stopping rule was more than a numerical accident.
%[text] The remaining question was whether we could turn this abstract stopping rule into something simple enough to do physically.
%%
%[text] ## The Marble Bag
%[text] Start with 1 red marble and 1 blue marble in the bag. Each round, draw a marble at random. If it is red, the game ends. If it is blue, put it back, add two more blue marbles, and draw again.
%[text] ![](flowchart.png)
%[text] Here is what happens:
%[text] - **Round 1:** Bag has 1 red + 1 blue (2 marbles). Draw. P(red) = 1/2.
%[text] - **Round 2:** Bag now has 1 red + 3 blue (4 marbles). Draw. P(red) = 1/4.
%[text] - **Round 3:** Bag now has 1 red + 5 blue (6 marbles). Draw. P(red) = 1/6.
%[text] - **Round k:** Bag has 1 red + (2k-1) blue = 2k marbles. P(red) = 1/(2k). \
%[text] Record the stopping round $\tau$. Compute the score: $\tau / (2\tau - 1)$. Repeat many times. The average score converges to $\pi/4$.
%[text] Claude generated a quick MATLAB simulation through the MCP server to check the estimator:
N = 10000;
tau_vals = zeros(N, 1);
for t = 1:N
    k = 1;
    while rand() >= 1/(2*k)
        k = k + 1;
    end
    tau_vals(t) = k;
end
scores = tau_vals ./ (2*tau_vals - 1);
running_est = 4 * cumsum(scores) ./ (1:N)';
fprintf('After 100 games:  pi ~ %.4f\n', running_est(100));
%%

semilogx(1:N, running_est, "b-", LineWidth=1.5); hold on
yline(pi, "r--", "\pi", LineWidth=2, FontSize=14, LabelHorizontalAlignment="left")
xlabel("Number of Games")
ylabel("Estimate of \pi")
title("Marble Bag: Convergence to \pi")
xlim([1 N]); ylim([2 5])
grid on
%%
%[text] With 100 games, the mean absolute error was about 0.06, or roughly 2% of $\pi$. The estimator is unbiased in expectation and does not depend on anyone having good aim with a dart.
%%
%[text] ## Proving It: The Symbolic Math Toolbox
%[text] I then used Claude Code with the Symbolic Math Toolbox to prove the result in closed form. I had not tried this workflow before; it produced the exact answer on the first attempt.
%%
%[text] ### The One-Line Proof
%[text] The stopping probability 1/(2k) at each round produces a PMF:
%[text] $P(\\tau = k) = \\frac{\\binom{2k-2}{k-1}}{k \\cdot 2^{2k-1}}$$$
%[text] Claude defined this symbolically and asked MATLAB for the expected score:
syms k
pmf = nchoosek(sym(2*k-2), k-1) / (k * 2^(2*k-1));
score = k / (2*k - 1);
% The one-line proof:
E_score = symsum(score * pmf, k, 1, inf)
%%
%[text] So with one call to *symsum*, the Symbolic Math Toolbox returns $\\pi/4$.
%%
%[text] ### Where Does Pi Come From?
%[text] The identity $\\tau/(2\\tau-1) = 1/2 + 1/(2(2\\tau-1))$ means:
%[text] $E\[\\tau/(2\\tau-1)\] = \\frac{1}{2} + \\frac{1}{2} \\cdot E\[1/(2\\tau-1)\]$.$$
%[text] The Symbolic Toolbox evaluates that inner expectation:
E_inv = symsum(pmf / (2*k - 1), k, 1, inf)
%%
%[text] This traces back to the arcsin series, one of the classical representations of $\\pi${"editStyle":"visual"}. The Symbolic Toolbox confirms both ingredients:

syms n
% The arcsin series (provides the pi):
symsum(nchoosek(sym(2*n),n) / (4^n*(2*n+1)), n, 0, inf)
%%

% The Catalan generating function (provides the offset):
symsum(nchoosek(sym(2*n),n) / (4^n*(n+1)), n, 0, inf)
%%
%[text] A partial fraction decomposition connects them:

syms x
partfrac(1/((x+1)*(2*x+1)), x)
%%
%[text] Combining:
%[text] $E\[\\tau/(2\\tau-1)\] = 1/2 + (1/2)(\\pi/2 - 1) = \\pi/4$$$
%[text] If you have not seen these before: the first series is the Taylor expansion of arcsin(x) at x = 1 (see [here](https://en.wikipedia.org/wiki/Inverse_trigonometric_functions#Infinite_series) for more detail). The second sums the Catalan numbers with geometric weights (see [here](https://oeis.org/A000108)). The partial fraction step shows how the score function decomposes into one piece from each series, which is where $\pi$ enters the calculation.
%%
%[text] ### How long will a game last?
%[text] We saw from our Monte Carlo simulations that the stopping round $\\tau$ is heavily right-skewed. You draw red on round 1 about half the time, but occasionally the game runs for 10+ rounds.
%%

k_theory = 1:15;
surv = zeros(1, 16);
for n = 0:15
    surv(n+1) = nchoosek(2*n, n) / 4^n;
end
pmf_theory = surv(1:end-1) - surv(2:end);

tau_counts = histcounts(tau_vals, 0.5:15.5);
bar(1:15, tau_counts/N, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'none'); hold on;
plot(k_theory, pmf_theory, 'ro-', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('Stopping Round (\tau)', 'FontSize', 13);
ylabel('Probability', 'FontSize', 13);
title('Distribution of Stopping Round', 'FontSize', 15);
legend('Simulation (N=10000)', 'Theory: P(\tau>n) = C(2n,n)/4^n', 'FontSize', 11);
grid on; set(gca, 'FontSize', 12);
%%
%[text] We asked Claude to verify that the PMF sums to 1 and explore the moments:
symsum(pmf, k, 1, inf)     % ans = 1    (valid PMF)
symsum(k * pmf, k, 1, inf) % ans = Inf  (infinite expected game length!)
%%
%[text] That second result matters. The game has infinite expected length, even though the score $\tau/(2\tau-1)$ has a finite mean.
%[text] This affects the physical version of the game. Most games finish quickly: 50% stop on round 1, and about 75% stop within 5 rounds. Occasionally, a game can keep going for a very long time. While discussing this post, Mike C had a simulated game that ran for 3,997,135,355 draws. With physical marbles, that would require about 40,000 metric tonnes of marbles and roughly 125 years.
%[text] I asked Claude whether there is an unbiased way to cap the game at a fixed number of rounds. There is: if red has not appeared by round M, stop and record a replacement score equal to the average score expected from the unfinished tail:
%[text] $c\_M = E\\left\[\\frac{\\tau}{2\\tau-1} \\mid \\tau \> M\\right\]$.$$
%[text] Five rounds is a practical cap. For M=5, the replacement score is approximately $c\_5 = 0.5162$.
%[text] So the practical classroom version works like this: draw at most 5 times. If you hit red, record $\\tau/(2\\tau-1)$. If you do not hit red in 5 draws, record 0.5162. About 25% of games are truncated, but the estimator remains unbiased because those unfinished games are replaced by their exact conditional average.
%%
%[text] ## What Made This Work
%[text] Three things made the result practical to find:
%[text] - **Claude Code for reasoning and brainstorming.** Claude proposed a non-obvious stopping rule and helped explore hazard-rate processes that I probably would not have tried first.
%[text] - **MATLAB via MCP for fast feedback.** Each idea could be tested in seconds, so weak candidates were discarded quickly and promising candidates were refined.
%[text] - **The Symbolic Math Toolbox for proof.** Moving from simulation to proof without switching tools turned the empirical hunch into a theorem. The *symsum* one-liner that returned $\pi/4$ was more convincing than another Monte Carlo run.
%[text] The result is a runnable classroom game with simulations, figures, and a closed-form proof.
%[text] The companion MATLAB scripts are in this folder.
