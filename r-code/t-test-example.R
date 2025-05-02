# Install required packages if not already installed
required_packages <- c("ggplot2", "gridExtra", "effsize")
new_packages <- required_packages[!required_packages %in% installed.packages()[,"Package"]]
if(length(new_packages)) install.packages(new_packages)

# Load packages
library(ggplot2)
library(gridExtra)

decentralized_times <- c(120, 135, 128, 142, 130)
centralized_times <- c(95, 105, 110, 98, 102)

# Calculate means and standard deviations
mean_decentralized <- mean(decentralized_times)
mean_centralized <- mean(centralized_times)
sd_decentralized <- sd(decentralized_times)
sd_centralized <- sd(centralized_times)

# Print summary statistics
cat("Decentralized method: Mean =", mean_decentralized, "seconds, SD =", sd_decentralized, "seconds\n")
cat("Centralized method: Mean =", mean_centralized, "seconds, SD =", sd_centralized, "seconds\n")
cat("Difference in means:", mean_decentralized - mean_centralized, "seconds\n\n")

# Basic t-test
t_result <- t.test(decentralized_times, centralized_times)
print(t_result)

# Calculate effect size (Cohen's d)
mean_diff <- abs(mean_decentralized - mean_centralized)
pooled_sd <- sqrt((sd_decentralized^2 + sd_centralized^2) / 2)
cohens_d <- mean_diff / pooled_sd

cat("\nEffect size (Cohen's d):", cohens_d, "\n")
if (cohens_d < 0.2) {
  cat("Interpretation: Negligible effect\n")
} else if (cohens_d < 0.5) {
  cat("Interpretation: Small effect\n")
} else if (cohens_d < 0.8) {
  cat("Interpretation: Medium effect\n")
} else {
  cat("Interpretation: Large effect\n")
}

if (require(ggplot2)) {
  # Combine data
  data <- data.frame(
    Time = c(decentralized_times, centralized_times),
    Method = c(rep("Decentralized", length(decentralized_times)), 
               rep("Centralized", length(centralized_times)))
  )
  
  # Create plot
  p <- ggplot(data, aes(x = Method, y = Time, fill = Method)) +
    geom_boxplot() +
    theme_minimal() +
    labs(title = "Comparison of Exploration Times",
         y = "Time to 90% Coverage (seconds)") +
    scale_fill_manual(values = c("Decentralized" = "coral", "Centralized" = "skyblue"))
  
  # Display plot
  print(p)
  
  # Add a more informative plot with means and error bars
  p2 <- ggplot(data, aes(x = Method, y = Time, fill = Method)) +
    stat_summary(fun = mean, geom = "bar") +
    stat_summary(fun.data = function(x) {
      return(c(y = mean(x), ymin = mean(x) - sd(x), ymax = mean(x) + sd(x)))
    }, geom = "errorbar", width = 0.2) +
    theme_minimal() +
    labs(title = "Mean Exploration Times with Standard Deviation",
         y = "Time to 90% Coverage (seconds)") +
    scale_fill_manual(values = c("Decentralized" = "coral", "Centralized" = "skyblue"))
  
  # Display second plot
  #print(p2)
}