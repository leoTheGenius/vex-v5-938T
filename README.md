Go to GitHub.com



Create account



Install GitHub Desktop



Clone the Repository





Committing and pushing updated code through terminal

    1. Save your work
    2. Go to the terminal
    3. Type "cd {wherever the repository is saved on your computer}"
        a. example: "cd Documents/vex-v5-938t/"
    4. Type "git status" and hit enter. you should see that your work has been changed.
    5. Type "git add ."
    6. Type "git commit -m "your commit message here""
        a. commit message should be helpful, describing what you have changed in the commit, such as "commit -m "changed buttons on controller""
    7. Then type "git push". this will update the github files.
        a. If there is an error in pushing the file, the file has probably gotten edited by another person. then you can type "git pull" and the changes will be saved to your computer.
        b. You might need an SSH key to push and pull commits.
            i. to create an SSH key, you will have to type into your terminal "ssh-keygen -t ed25519 -C "your_email@example.com""
            ii. it will prompt you to choose a file location. just use the default.
            iii. Then choose a password. you can leave this password empty.

IF THERE IS ANY PROBLEM, SEARCH IT UP FOR MORE INFORMATION.
