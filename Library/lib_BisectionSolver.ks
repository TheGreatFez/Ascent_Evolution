function makeBiSectSolver {
  parameter ScoreFunction, TestPoint1, TestPoint2.
  //
  local TestPoints is LIST().
  local Score1 is ScoreFunction:call(TestPoint1).
  local Score2 is ScoreFunction:call(TestPoint2).
  local TestPoint3 is (TestPoint1 + TestPoint2)/2.
  local Score3 is ScoreFunction:call(TestPoint3).

  TestPoints:ADD(LIST(TestPoint1,Score1)).
  TestPoints:ADD(LIST(TestPoint2,Score2)).
  TestPoints:ADD(LIST(TestPoint3,Score3)).
  print "Initial".
  print TestPoints.

  local expanded is true.
  until expanded {
    if TestPoints[0][1]*TestPoints[1][1] < 0 {
      set expanded to true.
      set TestPoints[2][0] to (TestPoints[0][0] + TestPoints[1][0])/2.
      set TestPoints[2][1] to ScoreFunction:call(TestPoints[2][0]).
      print "Expansion Finished".
      print TestPoints.
    } else {
      if abs(TestPoints[0][1]) < abs(TestPoints[1][1]) {
        local tmp to TestPoints[0][0] - TestPoints[1][0].
        set TestPoints[1][0] to TestPoints[0][0].
        set TestPoints[1][1] to TestPoints[0][1].
        set TestPoints[0][0] to TestPoints[0][0] + 1.5*tmp.
        set TestPoints[0][1] to ScoreFunction:call(TestPoints[0][0]).
      } else {
        local tmp to TestPoints[1][0] - TestPoints[0][0].
        set TestPoints[0][0] to TestPoints[1][0].
        set TestPoints[0][1] to TestPoints[1][1].
        set TestPoints[1][0] to TestPoints[1][0] + 1.5*tmp.
        set TestPoints[1][1] to ScoreFunction:call(TestPoints[1][0]).
      }
      print "Expanding".
      print TestPoints.
    }
  }

  return {
    // Re-test points
    set TestPoints[0][1] to ScoreFunction:call(TestPoints[0][0]).
    set TestPoints[1][1] to ScoreFunction:call(TestPoints[1][0]).
    set TestPoints[2][1] to ScoreFunction:call(TestPoints[2][0]).

    if TestPoints[0][1]*TestPoints[1][1] < 0 {
      if TestPoints[2][1]*TestPoints[0][1] > 0 {
        set TestPoints[0][0] to TestPoints[2][0].
      } else {
        set TestPoints[1][0] to TestPoints[2][0].
      }

      set TestPoints[2][0] to (TestPoints[0][0] + TestPoints[1][0])/2.
      set TestPoints[2][1] to ScoreFunction:call(TestPoints[2][0]).

    } else {
      // Expand the search area
      if abs(TestPoints[0][1]) < abs(TestPoints[1][1]) {
        local tmp to TestPoints[0][0] - TestPoints[1][0].
        set TestPoints[1][0] to TestPoints[0][0].
        set TestPoints[1][1] to TestPoints[0][1].
        set TestPoints[0][0] to TestPoints[0][0] + 1.5*tmp.
        set TestPoints[0][1] to ScoreFunction:call(TestPoints[0][0]).
      } else {
        local tmp to TestPoints[1][0] - TestPoints[0][0].
        set TestPoints[0][0] to TestPoints[1][0].
        set TestPoints[0][1] to TestPoints[1][1].
        set TestPoints[1][0] to TestPoints[1][0] + 1.5*tmp.
        set TestPoints[1][1] to ScoreFunction:call(TestPoints[1][0]).
      }

      set TestPoints[2][0] to (TestPoints[0][0] + TestPoints[1][0])/2.
      set TestPoints[2][1] to ScoreFunction:call(TestPoints[2][0]).
    }

    return TestPoints.
  }.
}
