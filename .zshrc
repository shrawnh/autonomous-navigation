# create a zsh command that will cp a file from a dir to the the same dir with different name depending on the first argument
function merge_model() {
    algorithm=$1
    alpha_model="/Users/shrwnh/Development/autonomous-navigation/src/simulation/controllers/$algorithm/${algorithm}_wheeled_robot-alpha.zip"
    stable_model="/Users/shrwnh/Development/autonomous-navigation/src/simulation/controllers/$algorithm/${algorithm}_wheeled_robot.zip"
    if [ ! -f "$alpha_model" ]; then
        echo -e "\033[31mAlpha model not found\033[0m"
        return
    fi
    if [ -f "$stable_model" ]; then
        echo "Do you want to merge the alpha model to the stable model? (y/n)"
        read answer
        if [ "$answer" != "${answer#[Yy]}" ]; then
            rm "$stable_model"
            cp "$alpha_model" "$stable_model"
            echo -e "\033[32mMerge successful\033[0m"
        else
          echo -e "\033[31mMerge aborted\033[0m"
        fi
    else
        cp "$alpha_model" "$stable_model"
        echo -e "\033[33mStable model created\033[0m"
    fi
}