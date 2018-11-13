rm(list = ls(all = TRUE))

library(ggplot2)
library(dplyr)
library(readr)
library(viridis)

## data_dir = "../data/2017-07-27_14-40-06"
## data_dir = "../data/2017-08-04_23-25-11"
## data_dir = "../data/2017-08-05_21-36-10"
## data_dir = "../data/2017-08-06_17-04-52"
data_dir = "../data/2017-08-07_22-26-30"
raw_data = read_delim(sprintf("%s/%s", data_dir, "outcomes.csv"),
                      delim = ",", trim_ws = TRUE,
                      col_types = cols(col_character(), ## pursuer
                                       col_integer(), ## level
                                       col_integer(), ## rep
                                       col_integer(), ## sim_num
                                       col_integer(), ## num_puruers
                                       col_double(), ## informant_rate
                                       col_integer(), ## rollout_steps
                                       col_integer(), ## time_points
                                       col_factor(levels=c("GOAL", "CAUGHT", "TIME")) ## outcome
                                       )
                      )

stopifnot(length(unique(raw_data$pursuer)) == 2)
stopifnot("qfn_rollout" %in% raw_data$pursuer)
stopifnot("rw" %in% raw_data$pursuer)

upper_bound <- function(x) {
  mean(x) + sqrt(var(x) / length(x))
}

lower_bound <- function(x) {
  mean(x) - sqrt(var(x) / length(x))
}

## plot probability of capture
capture_data_qfn = subset(raw_data, raw_data$pursuer == "qfn_rollout")
capture_data_qfn$was_caught = capture_data_qfn$outcome == "CAUGHT"
capture_data_qfn = summarise(group_by(capture_data_qfn, pursuer, num_pursuers,
                                      informant_rate, rollout_steps),
                             capture_prob = mean(was_caught),
                             upper_prob = upper_bound(was_caught),
                             lower_prob = lower_bound(was_caught))


capture_data_rw = subset(raw_data, raw_data$pursuer == "rw")
capture_data_rw$was_caught = capture_data_rw$outcome == "CAUGHT"
capture_data_rw = summarise(group_by(capture_data_rw, pursuer, num_pursuers),
                            capture_prob = mean(was_caught))

dup_levels = expand.grid(informant_rate = unique(capture_data_qfn$informant_rate),
                         rollout_steps = unique(capture_data_qfn$rollout_steps))
capture_data_rw_dup = NULL
for(i in 1:nrow(dup_levels)) {
  ## get levels
  informant_rate = dup_levels$informant_rate[i]
  rollout_steps = dup_levels$rollout_steps[i]
  ## create columns
  capture_data_rw$informant_rate = informant_rate
  capture_data_rw$rollout_steps = rollout_steps
  ## append rows
  capture_data_rw_dup = rbind(capture_data_rw_dup, capture_data_rw)
  ## clear columns
  capture_data_rw$informant_rate = NULL
  capture_data_rw$rollout_steps = NULL
}

capture_data = rbind(capture_data_rw_dup, capture_data_qfn)


p = ggplot() +
  geom_line(data = capture_data, aes(x = rollout_steps, y = capture_prob,
                                     color = as.factor(num_pursuers),
                                     lty = pursuer
                                     )
            ) +
  geom_errorbar(data = capture_data,
                aes(x = rollout_steps,
                    ymin = lower_prob, ymax = upper_prob,
                    color = as.factor(num_pursuers)), width = 0.1) +
  facet_wrap(~informant_rate, nrow = 1,
             labeller = label_bquote(lambda == .(informant_rate))) +
  scale_linetype_discrete("Search strategy",
                          breaks = c("qfn_rollout", "rw"),
                          labels = c("Thompson sampling", "Random walk")) +
  scale_color_discrete("Number of pursuers") +
  theme(panel.spacing = unit(1.5, "lines"),
        legend.position = "bottom") +
  xlab(expression(paste("Time points before heuristic: ", n))) +
  ylab("Estimated probability of capture") +
  guides(linetype = guide_legend(order = 1),
         color = guide_legend(order = 2))

print(p)


ggsave("../data/figures/prob_capture.pdf", p)
ggsave("../data/figures/prob_capture.svg", p)
