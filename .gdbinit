define reset
  monitor reset
  continue
end

define flash
  load
  reset
end

define r
  target remote $arg0:3333
  #monitor semihosting enable
end
