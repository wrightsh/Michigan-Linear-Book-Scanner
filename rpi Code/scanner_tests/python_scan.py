

import pexpect

command = 'sudo scanimage --format=tiff --batch="my_batch_scan_2_%d.tiff" --batch-prompt'

scanner_1 = pexpect.spawn(command)

scanner_1.expect('Press <RETURN> to continue.')
scanner_1.sendline('')

print "Alive: ", scanner_1.isalive()

scanner_1.expect('Press <RETURN> to continue.')
scanner_1.sendline('')

print "Alive: ", scanner_1.isalive()

scanner_1.expect('Press <RETURN> to continue.')

print "Alive: ", scanner_1.isalive()

