DefinitionBlock("DSDT.aml", "DSDT", 5, "SAMSUN", "EXY7885 ", 2)
{
	Scope (\_SB_)
  {
        /* 2 CPU Cores */
		Device (CPU0)
        {
		    Name (_HID, "ACPI0007")
            Name (_UID, 0x0)
			Method (_STA)
            {
                Return(0xf)
            }
        }
  }
}