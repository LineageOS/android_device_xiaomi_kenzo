1 To run this suite separately(each test case creates table and delete table) use below command
   - To execute test suite nt times with n entries, command "ipanatest sep nt n"

  Example:  To execute test suite 1 time with 100 entries, command "ipanattest sep 100"


2. To run test suite not separately(creates table and delete table only once) use below command
   - To execute test suite nt times with n entries, command "ipanatest reg nt n"

   Example: To execute test suite 5 times with 32 entries, command "ipanattest reg 5 32"


3. To run inotify regression test use command, "ipanattest inotify nt"

   Example: To execute inotify 5 times, command "ipanattest inotify 5"


4. if we just give command "ipanattest", runs test suite 1 time with 100 entries (non separate)
