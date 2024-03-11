zimfw() { source /home/p76124605/.zim/zimfw.zsh "${@}" }
zmodule() { source /home/p76124605/.zim/zimfw.zsh "${@}" }
fpath=(/home/p76124605/.zim/modules/git/functions /home/p76124605/.zim/modules/utility/functions /home/p76124605/.zim/modules/duration-info/functions /home/p76124605/.zim/modules/git-info/functions /home/p76124605/.zim/modules/zsh-completions/src ${fpath})
autoload -Uz -- git-alias-lookup git-branch-current git-branch-delete-interactive git-branch-remote-tracking git-dir git-ignore-add git-root git-stash-clear-interactive git-stash-recover git-submodule-move git-submodule-remove mkcd mkpw duration-info-precmd duration-info-preexec coalesce git-action git-info
source /home/p76124605/.zim/modules/environment/init.zsh
source /home/p76124605/.zim/modules/git/init.zsh
source /home/p76124605/.zim/modules/input/init.zsh
source /home/p76124605/.zim/modules/termtitle/init.zsh
source /home/p76124605/.zim/modules/utility/init.zsh
source /home/p76124605/.zim/modules/duration-info/init.zsh
source /home/p76124605/.zim/modules/asciiship/asciiship.zsh-theme
source /home/p76124605/.zim/modules/completion/init.zsh
source /home/p76124605/.zim/modules/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh
source /home/p76124605/.zim/modules/zsh-history-substring-search/zsh-history-substring-search.zsh
source /home/p76124605/.zim/modules/zsh-autosuggestions/zsh-autosuggestions.zsh
source /home/p76124605/.zim/modules/powerlevel10k/powerlevel10k.zsh-theme
