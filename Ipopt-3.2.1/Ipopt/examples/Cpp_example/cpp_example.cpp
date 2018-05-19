// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Common Public License.
//
// $Id: cpp_example.cpp 759 2006-07-07 03:07:08Z andreasw $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "MyNLP.hpp"

using namespace Ipopt;

int main(int argv, char* argc[])
{
  // Create an instance of your nlp...
  SmartPtr<TNLP> mynlp = new MyNLP();

  // Create an instance of the IpoptApplication
  SmartPtr<IpoptApplication> app = new IpoptApplication();

  // Initialize the IpoptApplication and process the options
  app->Initialize();

  ApplicationReturnStatus status = app->OptimizeTNLP(mynlp);

  if (status == Solve_Succeeded) {
    // Retrieve some statistics about the solve
    Index iter_count = app->Statistics()->IterationCount();
    printf("\n\n*** The problem solved in %d iterations!\n", iter_count);

    Number final_obj = app->Statistics()->FinalObjective();
    printf("\n\n*** The final value of the objective function is %e.\n", final_obj);
  }

  return (int) status;
}
