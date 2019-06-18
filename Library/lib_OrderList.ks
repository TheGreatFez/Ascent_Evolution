function OrderList {
  parameter InputList, Ascending.
  local Ordered is false.
  until Ordered {
    local fixes is false.
    from {local item is 0.} until (item = InputList:length - 1) step {set item to item + 1.} do {
      if Ascending {
        if InputList[item] > InputList[item + 1] {
          local temp is InputList[item + 1].
          set InputList[item + 1] to InputList[item].
          set InputList[item] to temp.
          set fixes to true.
        }
      } else {
        if InputList[item] < InputList[item + 1] {
          local temp is InputList[item + 1].
          set InputList[item + 1] to InputList[item].
          set InputList[item] to temp.
          set fixes to true.
        }
      }
    }
    if not(fixes) {
      set Ordered to true.
    }
  }

  return InputList.
}
