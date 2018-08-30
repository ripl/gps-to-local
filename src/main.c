#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include <getopt.h>
#include <glib.h>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include <hr_common/nmea.h>


#include <lcmtypes/senlcm_nmea_t.h>
#include <lcmtypes/hr_lcmtypes.h>

// Used if we are publishing the pose
#define PUBLISH_GLOBAL_TO_LOCAL_HZ 10.0


#if DEBUG
#define dbg(...) do { fprintf(stderr, "[%s:%d] ", __FILE__, __LINE__); \
                      fprintf(stderr, __VA_ARGS__); } while(0)
#else
#define dbg(...)
#endif


#ifndef SQUARE
#define SQUARE(a) ( (a)*(a) )
#endif

typedef struct _state_t {
    lcm_t * lcm;
    BotParam * param;
    BotFrames * frames;

    GMainLoop *mainloop;

    BotTrans gps_to_body_trans;

    ripl_gps_to_local_t gps_to_local;
    bot_core_pose_t pose;

    int verbose;
    int publish_global_to_local;


} state_t;



static void
on_nmea (const lcm_recv_buf_t *rbuf, const char *channel,
         const senlcm_nmea_t *nmea, void *user )
{
    state_t *self = (state_t *) user;

    gps_state_t *gps = (gps_state_t *) calloc (1, sizeof(gps_state_t));

    if (nmea_parse (gps, nmea) != 1) {
        if (self->verbose)
            fprintf (stdout, "Error parsing NMEA message\n");

        free (gps);
        return;
      }

   self->gps_to_local.utime = gps->recv_utime;
   self->gps_to_local.lat_lon_el_theta[0] = gps->lat;
	 self->gps_to_local.lat_lon_el_theta[1] = gps->lon;
	 self->gps_to_local.lat_lon_el_theta[2] = gps->elev;
	 self->gps_to_local.lat_lon_el_theta[3] = 0.0;


   BotTrans *pose = calloc(1,sizeof(BotTrans));

   if (bot_frames_get_trans(self->frames, "body", "local", pose)) {
     self->gps_to_local.local[0] = pose->trans_vec[0];
     self->gps_to_local.local[1] = pose->trans_vec[1];
     self->gps_to_local.local[2] = pose->trans_vec[2];

     ripl_gps_to_local_t_publish (self->lcm, "GPS_TO_LOCAL", &self->gps_to_local);

   }

}



static void
usage (int argc, char ** argv)
{
    fprintf (stderr, "Usage: %s [OPTIONS]\n"
             "Publish GPS_TO_LOCAL based on GPS18 NMEA...\n"
             "\n"
             "Options:\n"
             "    -p, --publish-global-to-local    Publish GLOBAL_TO_LOCAL message\n"
             "    -v     Be verbose\n"
             "    -h     This help message\n", argv[0]);
};


int
main (int argc, char ** argv)
{

    // so that redirected stdout won't be insanely buffered.
    setlinebuf(stdout);


    state_t *self = (state_t *) calloc (1, sizeof (state_t));
    int c;

    char *optstring = "hvp";
    struct option long_opts[] = {
        { "help", no_argument, NULL, 'h' },
        { "verbose", no_argument, NULL, 'v' },
        { "publish-global-to-local", no_argument, NULL, 'p' },
        { 0, 0, 0, 0 }
    };

    while ((c = getopt_long (argc, argv, optstring, long_opts, 0 )) >= 0) {
        switch (c) {
        case 'v':
            self->verbose = 1;
            break;
        case 'p':
            self->publish_global_to_local = 1;
            break;
        case 'h':
        case '?':
        default:
            usage(argc, argv);
            return 1;
        }
    }

    self->lcm = bot_lcm_get_global (NULL);
    if (!self->lcm) {
      fprintf (stderr, "Error getting LCM\n");
      free (self);
      return -1;
    }

    self->param = bot_param_new_from_server (self->lcm, 0);
    if (!self->param) {
      fprintf (stderr, "Error getting BotParam\n");
      free (self);
      return -1;
    }

    self->frames = bot_frames_get_global (self->lcm, self->param);
    if (!self->frames) {
      fprintf (stderr, "Error getting BotFrames\n");
      free (self);
      return -1;
    }

    if (bot_frames_get_trans (self->frames, "gps18", "body", &(self->gps_to_body_trans)) == 0) {
        fprintf (stderr, "Error getting transformation from GPS to BODY via BotFrames\n");
        return -1;
    }

    senlcm_nmea_t_subscribe( self->lcm, "NMEA", on_nmea, self);

    /* Main Loop */
    self->mainloop = g_main_loop_new(NULL, FALSE);
    if (!self->mainloop) {
        fprintf(stderr,"Error: Failed to create the main loop\n");
        return -1;
    }

    // attach LCM to the mainloop
    bot_glib_mainloop_attach_lcm (self->lcm);


    /* sit and wait for messages */
    g_main_loop_run(self->mainloop);

    free (self);
    return 0;
}
