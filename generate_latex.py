import os

preamble = r"""\documentclass{article}
\usepackage[utf8]{inputenc}
\usepackage{amsmath}
\usepackage{graphicx}
\usepackage{listings}
\usepackage{xcolor}
\usepackage{geometry}
\geometry{margin=1in}

\definecolor{codegreen}{rgb}{0,0.6,0}
\definecolor{codegray}{rgb}{0.5,0.5,0.5}
\definecolor{codepurple}{rgb}{0.58,0,0.82}
\definecolor{backcolour}{rgb}{0.95,0.95,0.92}

\lstdefinestyle{mystyle}{
    backgroundcolor=\color{backcolour},
    commentstyle=\color{codegreen},
    keywordstyle=\color{magenta},
    numberstyle=\tiny\color{codegray},
    stringstyle=\color{codepurple},
    basicstyle=\ttfamily\footnotesize,
    breakatwhitespace=false,
    breaklines=true,
    captionpos=b,
    keepspaces=true,
    numbers=left,
    numbersep=5pt,
    showspaces=false,
    showstringspaces=false,
    showtabs=false,
    tabsize=2
}
\lstset{style=mystyle}

\begin{document}
"""

def read_file(filepath):
    with open(filepath, 'r') as f:
        return f.read()

# Mini_Project3.tex
content3 = preamble + r"""
\title{Mini Project 3}
\author{EECE 7398 Legged Robotics}
\date{}
\maketitle

\section*{Code Listing}

\subsection*{\texttt{func\_zero\_dynamics.m}}
\begin{lstlisting}[language=Matlab]
""" + read_file('func_zero_dynamics.m') + r"""
\end{lstlisting}

\subsection*{\texttt{Optimize.m}}
\begin{lstlisting}[language=Matlab]
""" + read_file('Optimize.m') + r"""
\end{lstlisting}

\subsection*{\texttt{sim\_and\_plot\_ZD.m}}
\begin{lstlisting}[language=Matlab]
""" + read_file('sim_and_plot_ZD.m') + r"""
\end{lstlisting}

\section*{Outputs}

\subsection*{Optimization Output (\texttt{Optimize.m})}
\begin{lstlisting}
Local minimum found that satisfies the constraints.

Optimization completed because the objective function is non-decreasing in 
feasible directions, to within the value of the optimality tolerance,
and constraints are satisfied to within the value of the constraint tolerance.

=== Final Solution ===
f   = [-0.4907, -2.3947, 0.3053, 0.6256, 0.8814, 2.6360, 2.7919, 2.9537]
Cost J_avg = 338.8789
Step length = 0.8520 m
\end{lstlisting}

\subsection*{Zero Dynamics Simulation (\texttt{sim\_and\_plot\_ZD.m})}
\begin{figure}[h!]
    \centering
    \includegraphics[width=0.8\textwidth]{ZD.png}
    \caption{Phase portrait of the zero dynamics ($q_1$ vs.\ $\dot{q}_1$).}
\end{figure}

\end{document}
"""

with open('Mini_Project3.tex', 'w') as f:
    f.write(content3)

# Mini_Project4.tex
content4 = preamble + r"""
\title{Mini Project 4}
\author{EECE 7398 Legged Robotics}
\date{}
\maketitle

\section*{Code Listing}

\subsection*{\texttt{func\_feedback.m}}
\begin{lstlisting}[language=Matlab]
""" + read_file('func_feedback.m') + r"""
\end{lstlisting}

\subsection*{\texttt{sim\_and\_plot\_full\_dynamics.m}}
\begin{lstlisting}[language=Matlab]
""" + read_file('sim_and_plot_full_dynamics.m') + r"""
\end{lstlisting}

\section*{Outputs}

\subsection*{Simulation Text Output}
\begin{lstlisting}
Step 1: q1_plus=0.3907, dq1_plus=-2.2327
Step#...1
Step 2: q1_plus=0.3907, dq1_plus=-2.2072
Step#...2
Step 3: q1_plus=0.3907, dq1_plus=-2.1927
Step#...3
Step 4: q1_plus=0.3907, dq1_plus=-2.1845
Step#...4
Step 5: q1_plus=0.3907, dq1_plus=-2.1801
Step#...5
Step 6: q1_plus=0.3907, dq1_plus=-2.1773
Step#...6
Step 7: q1_plus=0.3907, dq1_plus=-2.1758
Step#...7
Step 8: q1_plus=0.3907, dq1_plus=-2.1749
Step#...8
Step 9: q1_plus=0.3907, dq1_plus=-2.1744
Step#...9
Step 10: q1_plus=0.3907, dq1_plus=-2.1741
Step#...10
Step 11: q1_plus=0.3907, dq1_plus=-2.1739
Step#...11
Step 12: q1_plus=0.3907, dq1_plus=-2.1738
Step#...12
Step 13: q1_plus=0.3907, dq1_plus=-2.1738
Step#...13
Step 14: q1_plus=0.3907, dq1_plus=-2.1738
Step#...14
Step 15: q1_plus=0.3907, dq1_plus=-2.1738
Step#...15

=== Mechanical Cost of Transport ===
Total dist   = 12.7806 m
Step length  = 0.8520 m
Total mass   = 35.0 kg
Walking speed= 1.7265 m/s
MCOT         = 1.1584
\end{lstlisting}

\clearpage
\subsection*{Simulation Plots}

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.9\textwidth]{q_dq.png}
    \caption{Joint angles and angular velocities.}
\end{figure}

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.9\textwidth]{FD.png}
    \caption{Phase portrait of the full dynamics ($q_1$ vs.\ $\dot{q}_1$).}
\end{figure}

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.9\textwidth]{anim_snapshot.png}
    \caption{Frame-by-frame animation snapshot of the walking gait.}
\end{figure}

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.9\textwidth]{pointcare_error.png}
    \caption{Step-to-step Poincar\'e map error showing convergence.}
\end{figure}

\end{document}
"""

with open('Mini_Project4.tex', 'w') as f:
    f.write(content4)
