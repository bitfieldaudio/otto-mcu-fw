define reset
  monitor reset halt
  continue
end

define flash
  load
  reset
end

define r
  target remote $arg0:3333
  monitor arm semihosting enable
end
